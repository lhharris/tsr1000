#include "tsr1000.h"

#include <functional>

#include <QCoreApplication>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLoggingCategory>
#include <QRegularExpression>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QStringList>
#include <QThread>
#include <QUrl>

#include "hexvalidator.h"
#include "link/seriallink.h"
#include "networkmanager.h"
#include "networktool.h"
#include "notificationmanager.h"
#include "settingsmanager.h"

PING_LOGGING_CATEGORY(PING_PROTOCOL_TSR1000, "ping.protocol.tsr1000")

const int Tsr1000::_pingMaxFrequency = 50;

const bool Tsr1000::_firmwareDefaultAutoMode = true;
const int Tsr1000::_firmwareDefaultGainSetting = 1;
const bool Tsr1000::_firmwareDefaultPingEnable = true;
const uint16_t Tsr1000::_firmwareDefaultPingInterval = 250;
const uint32_t Tsr1000::_firmwareDefaultSpeedOfSound = 1500;

Tsr1000::Tsr1000()
    : PingSensor(PingDeviceType::TSR1000)
    , _points(_num_points, 0)
{
    _flasher = new Flasher(nullptr);

    setName("TSR1000");
    setControlPanel({"qrc:/Tsr1000ControlPanel.qml"});
    setSensorVisualizer({"qrc:/Tsr1000Visualizer.qml"});
    setSensorStatusModel({"qrc:/Tsr1000StatusModel.qml"});

    _periodicRequestTimer.setInterval(1000);
    connect(&_periodicRequestTimer, &QTimer::timeout, this, [this] {
        if (!link()->isWritable()) {
            qCWarning(PING_PROTOCOL_TSR1000) << "Can't write in this type of link.";
            _periodicRequestTimer.stop();
            return;
        }

        if (!link()->isOpen()) {
            qCCritical(PING_PROTOCOL_TSR1000) << "Can't write, port is not open!";
            _periodicRequestTimer.stop();
            return;
        }

        // Update lost messages count
        _lostMessages = 0;
        for (const auto& requestedId : requestedIds) {
            _lostMessages += requestedId.waiting;
        }
        emit lostMessagesChanged();

        if (!_commonVariables.deviceInformation.initialized) {
            request(CommonId::DEVICE_INFORMATION);
        }

        request(Tsr1000Id::PCB_TEMPERATURE);
        request(Tsr1000Id::PROCESSOR_TEMPERATURE);
        request(Tsr1000Id::VOLTAGE_5);
        request(Tsr1000Id::MODE_AUTO);
    });

    // connectLink(LinkType::Serial, {"/dev/ttyUSB2", "115200"});

    connect(this, &Sensor::connectionOpen, this, &Tsr1000::startPreConfigurationProcess);

    // Wait for device id to load the correct settings
    connect(this, &Tsr1000::srcIdChanged, this, &Tsr1000::setLastPingConfiguration);

    connect(this, &Tsr1000::deviceRevisionChanged, this, [this] {
        // Wait for firmware information to be available before looking for new versions
        //static bool once = false;
        static bool once = true;  // don't go looking for new firmware right now.
        if (!once) {
            once = true;
            //QString sensorName = _commonVariables.deviceInformation.device_revision == 2 ? "ping2" : "tsr1000";
            QString sensorName = "TSR1000";
            NetworkTool::self()->checkNewFirmware(
                sensorName, std::bind(&Tsr1000::checkNewFirmwareInGitHubPayload, this, std::placeholders::_1));
        }
    });
}

void Tsr1000::startPreConfigurationProcess()
{
    qCDebug(PING_PROTOCOL_TSR1000) << "Start pre configuration task and requests.";
    if (!link()->isWritable()) {
        qCDebug(PING_PROTOCOL_TSR1000) << "It's only possible to set last configuration when link is writable.";
        return;
    }

    // Request device information
    request(Tsr1000Id::PING_ENABLE);
    request(Tsr1000Id::MODE_AUTO);
    request(Tsr1000Id::PROFILE);
    request(CommonId::DEVICE_INFORMATION); // We should use this over Tsr1000Id::FIRMWARE_VERSION
    request(Tsr1000Id::DEVICE_ID);
    request(Tsr1000Id::SPEED_OF_SOUND);

    // Start periodic request timer
    _periodicRequestTimer.start();
}

void Tsr1000::loadLastPingConfigurationSettings()
{
    // Set default values
    for (const auto& key : _pingConfiguration.keys()) {
        _pingConfiguration[key].value = _pingConfiguration[key].defaultValue;
    }

    // Load settings for device using device id
    QVariant pingConfigurationVariant
        = SettingsManager::self()->getMapValue({"TSR1000", "Tsr1000Configuration", QString(_commonVariables.srcId)});
    if (pingConfigurationVariant.type() != QVariant::Map) {
        qCWarning(PING_PROTOCOL_TSR1000) << "No valid Tsr1000Configuration in settings." << pingConfigurationVariant.type();
        return;
    }

    // Get the value of each configuration and set it on device
    auto map = pingConfigurationVariant.toMap();
    for (const auto& key : _pingConfiguration.keys()) {
        _pingConfiguration[key].set(map[key]);
    }
}

void Tsr1000::updatePingConfigurationSettings()
{
    // Save all sensor configurations
    for (const auto& key : _pingConfiguration.keys()) {
        auto& dataStruct = _pingConfiguration[key];
        dataStruct.set(dataStruct.getClassValue());
        SettingsManager::self()->setMapValue(
            {"TSR1000", "Tsr1000Configuration", QString(_commonVariables.srcId), key}, dataStruct.value);
    }
}

void Tsr1000::connectLink(LinkType connType, const QStringList& connString)
{
    Sensor::connectLink(LinkConfiguration {connType, connString});
    startPreConfigurationProcess();
}

void Tsr1000::handleMessage(const ping_message& msg)
{
    qCDebug(PING_PROTOCOL_TSR1000) << QStringLiteral("Handling Message: %1 [%2]")
                                       .arg(PingHelper::nameFromMessageId(
                                           static_cast<PingEnumNamespace::PingMessageId>(msg.message_id())))
                                       .arg(msg.message_id());

    auto& requestedId = requestedIds[msg.message_id()];
    if (requestedId.waiting) {
        requestedId.waiting--;
        requestedId.ack++;
    }

    switch (msg.message_id()) {

    // This message is deprecated, it provides no added information because
    // the device id is already supplied in every message header
    case Tsr1000Id::DEVICE_ID: {
        tsr1000_device_id m(msg);
        _commonVariables.srcId = m.source_device_id();

        emit srcIdChanged();
    } break;

    case Tsr1000Id::DISTANCE: {
        tsr1000_distance m(msg);
        _distance = m.distance();
        _confidence = m.confidence();
        _transmit_duration = m.transmit_duration();
        _ping_number = m.ping_number();
        _scan_start = m.scan_start();
        _scan_length = m.scan_length();
        _gain_setting = m.gain_setting();

        emit distanceChanged();
        emit pingNumberChanged();
        emit confidenceChanged();
        emit transmitDurationChanged();
        emit scanStartChanged();
        emit scanLengthChanged();
        emit gainSettingChanged();
    } break;

    case Tsr1000Id::DISTANCE_SIMPLE: {
        tsr1000_distance_simple m(msg);
        _distance = m.distance();
        _confidence = m.confidence();

        emit distanceChanged();
        emit confidenceChanged();
    } break;

    case Tsr1000Id::PROFILE: {
        tsr1000_profile m(msg);
        _distance = m.distance();
        _confidence = m.confidence();
        _transmit_duration = m.transmit_duration();
        _ping_number = m.ping_number();
        _scan_start = m.scan_start();
        _scan_length = m.scan_length();
        _gain_setting = m.gain_setting();
        _num_points = m.profile_data_length();

        if (_num_points != _points.size()) {
            _points.resize(_num_points);
        }
#pragma omp for
        // This is necessary to convert <uint8_t> to <int>
        // QProperty only supports vector<int>, otherwise, we could use memcpy, like the two lines below:
        // _num_points = m.profile_data_length(); // const for no
        // memcpy(_points.data(), m.data(), _num_points); // careful with constant
        for (int i = 0; i < m.profile_data_length(); i++) {
            _points.replace(i, m.profile_data()[i] / 65535.0);
        }

        emit distanceChanged();
        emit pingNumberChanged();
        emit confidenceChanged();
        emit transmitDurationChanged();
        emit scanStartChanged();
        emit scanLengthChanged();
        emit gainSettingChanged();
        emit pointsChanged();
    } break;

    case Tsr1000Id::MODE_AUTO: {
        tsr1000_mode_auto m(msg);
        if (_mode_auto != static_cast<bool>(m.mode_auto())) {
            _mode_auto = m.mode_auto();
            emit modeAutoChanged();
        }
    } break;

    case Tsr1000Id::PING_ENABLE: {
        tsr1000_ping_enable m(msg);
        _ping_enable = m.ping_enabled();
        emit pingEnableChanged();
    } break;

    case Tsr1000Id::PING_INTERVAL: {
        tsr1000_ping_interval m(msg);
        _ping_interval = m.ping_interval();
        emit pingIntervalChanged();
    } break;

    case Tsr1000Id::RANGE: {
        tsr1000_range m(msg);
        _scan_start = m.scan_start();
        _scan_length = m.scan_length();
        emit scanLengthChanged();
        emit scanStartChanged();
    } break;

    case Tsr1000Id::GENERAL_INFO: {
        tsr1000_general_info m(msg);
        _gain_setting = m.gain_setting();
        emit gainSettingChanged();
    } break;

    case Tsr1000Id::GAIN_SETTING: {
        tsr1000_gain_setting m(msg);
        _gain_setting = m.gain_setting();
        emit gainSettingChanged();
    } break;

    case Tsr1000Id::SPEED_OF_SOUND: {
        tsr1000_speed_of_sound m(msg);
        _speed_of_sound = m.speed_of_sound();
        emit speedOfSoundChanged();
    } break;

    case Tsr1000Id::PROCESSOR_TEMPERATURE: {
        tsr1000_processor_temperature m(msg);
        _processor_temperature = m.processor_temperature();
        emit processorTemperatureChanged();
        break;
    }

    case Tsr1000Id::PCB_TEMPERATURE: {
        tsr1000_pcb_temperature m(msg);
        _pcb_temperature = m.pcb_temperature();
        emit pcbTemperatureChanged();
        break;
    }

    case Tsr1000Id::VOLTAGE_5: {
        tsr1000_voltage_5 m(msg);
        _board_voltage = m.voltage_5(); // millivolts
        emit boardVoltageChanged();
        break;
    }

    default:
        qWarning(PING_PROTOCOL_TSR1000) << "UNHANDLED MESSAGE ID:" << msg.message_id();
        break;
    }

    emit parsedMsgsChanged();
}

void Tsr1000::firmwareUpdate(QString fileUrl, bool sendPingGotoBootloader, int baud, bool verify)
{
    if (fileUrl.contains("http")) {
        NetworkManager::self()->download(fileUrl, [this, sendPingGotoBootloader, baud, verify](const QString& path) {
            qCDebug(FLASH) << "Downloaded firmware:" << path;
            flash(path, sendPingGotoBootloader, baud, verify);
        });
    } else {
        flash(fileUrl, sendPingGotoBootloader, baud, verify);
    }
}

void Tsr1000::flash(const QString& fileUrl, bool sendPingGotoBootloader, int baud, bool verify)
{
    flasher()->setState(Flasher::Idle);
    flasher()->setState(Flasher::StartingFlash);
    if (!HexValidator::isValidFile(fileUrl)) {
        auto errorMsg = QStringLiteral("File does not contain a valid Intel Hex format: %1").arg(fileUrl);
        qCWarning(PING_PROTOCOL_TSR1000) << errorMsg;
        flasher()->setState(Flasher::Error, errorMsg);
        return;
    };

    SerialLink* serialLink = dynamic_cast<SerialLink*>(link());
    if (!serialLink) {
        auto errorMsg = QStringLiteral("It's only possible to flash via serial.");
        qCWarning(PING_PROTOCOL_TSR1000) << errorMsg;
        flasher()->setState(Flasher::Error, errorMsg);
        return;
    }

    if (!link()->isOpen()) {
        auto errorMsg = QStringLiteral("Link is not open to do the flash procedure.");
        qCWarning(PING_PROTOCOL_TSR1000) << errorMsg;
        flasher()->setState(Flasher::Error, errorMsg);
        return;
    }

    // Stop requests and messages from the sensor
    _periodicRequestTimer.stop();
    setPingFrequency(0);

    if (sendPingGotoBootloader) {
        qCDebug(PING_PROTOCOL_TSR1000) << "Put it in bootloader mode.";
        tsr1000_goto_bootloader m;
        m.updateChecksum();
        writeMessage(m);
    }

    // Wait for bytes to be written before finishing the connection
    while (serialLink->port()->bytesToWrite()) {
        qCDebug(PING_PROTOCOL_TSR1000) << "Waiting for bytes to be written...";
        // We are not changing the connection structure, only waiting for bytes to be written
        const_cast<QSerialPort*>(serialLink->port())->waitForBytesWritten();
        qCDebug(PING_PROTOCOL_TSR1000) << "Done !";
    }

    qCDebug(PING_PROTOCOL_TSR1000) << "Finish connection.";

    auto flashSensor = [=] {
        flasher()->setBaudRate(baud);
        flasher()->setFirmwarePath(fileUrl);
        flasher()->setLink(link()->configuration()[0]);
        flasher()->setVerify(verify);
        flasher()->flash();
    };

    auto finishConnection = [=] {
        link()->finishConnection();

        QSerialPortInfo pInfo(serialLink->port()->portName());
        QString portLocation = pInfo.systemLocation();

        qCDebug(PING_PROTOCOL_TSR1000) << "Save sensor configuration.";
        updatePingConfigurationSettings();

        qCDebug(PING_PROTOCOL_TSR1000) << "Start flash.";

        QTimer::singleShot(1000, flashSensor);
    };
    QTimer::singleShot(1000, finishConnection);

    // Clear last configuration src ID to detect device as a new one
    connect(_flasher, &Flasher::stateChanged, this, [this] {
        if (flasher()->state() == Flasher::States::FlashFinished) {
            QThread::msleep(500);
            // Clear last configuration src ID to detect device as a new one
            resetSensorLocalVariables();
            Sensor::connectLink(*link()->configuration());
        }
    });
}

void Tsr1000::setLastPingConfiguration()
{
    if (_lastPingConfigurationSrcId == _commonVariables.srcId) {
        return;
    }
    _lastPingConfigurationSrcId = _commonVariables.srcId;
    if (!link()->isWritable()) {
        qCDebug(PING_PROTOCOL_TSR1000) << "It's only possible to set last configuration when link is writable.";
        return;
    }

    // Load previous configuration with device id
    loadLastPingConfigurationSettings();

    // Request at least a single profile to get device configuration
    emitPing();

    // Print last configuration
    QString output = QStringLiteral("\nTsr1000Configuration {\n");
    for (const auto& key : _pingConfiguration.keys()) {
        output += QString("\t%1: %2\n").arg(key).arg(_pingConfiguration[key].value);
    }
    output += QStringLiteral("}");
    qCDebug(PING_PROTOCOL_TSR1000).noquote() << output;

    // Set loaded configuration in device
    static QString debugMessage
        = QStringLiteral("Device configuration does not match. Waiting for (%1), got (%2) for %3");
    static auto lastPingConfigurationTimer = new QTimer();
    connect(lastPingConfigurationTimer, &QTimer::timeout, this, [this] {
        bool stopLastPingConfigurationTimer = true;
        for (const auto& key : _pingConfiguration.keys()) {
            auto& dataStruct = _pingConfiguration[key];
            if (dataStruct.value != dataStruct.getClassValue()) {
                qCDebug(PING_PROTOCOL_TSR1000)
                    << debugMessage.arg(dataStruct.value).arg(dataStruct.getClassValue()).arg(key);
                dataStruct.setClassValue(dataStruct.value);
                stopLastPingConfigurationTimer = false;
            }
            if (key.contains("automaticMode") && dataStruct.value) {
                qCDebug(PING_PROTOCOL_TSR1000) << "Device was running with last configuration in auto mode.";
                // If it's running in automatic mode
                // no further configuration is necessary
                break;
            }
        }
        if (stopLastPingConfigurationTimer) {
            qCDebug(PING_PROTOCOL_TSR1000) << "Last configuration done, timer will stop now.";
            lastPingConfigurationTimer->stop();
            do_continuous_start(ContinuousId::PROFILE);
        }
    });
    lastPingConfigurationTimer->start(500);
    lastPingConfigurationTimer->start();
}

void Tsr1000::setPingFrequency(float pingFrequency)
{
    if (pingFrequency <= 0 || pingFrequency > _pingMaxFrequency) {
        qCWarning(PING_PROTOCOL_TSR1000) << "Invalid frequency:" << pingFrequency;
        do_continuous_stop(ContinuousId::PROFILE);
    } else {
        int periodMilliseconds = 1000.0f / pingFrequency;
        qCDebug(PING_PROTOCOL_TSR1000) << "Setting frequency(Hz) and period(ms):" << pingFrequency << periodMilliseconds;
        set_ping_interval(periodMilliseconds);
        do_continuous_start(ContinuousId::PROFILE);
    }
    qCDebug(PING_PROTOCOL_TSR1000) << "Ping frequency" << pingFrequency;
}

void Tsr1000::resetSettings()
{
    qCDebug(PING_PROTOCOL_TSR1000) << "Settings will be reseted.";
    set_speed_of_sound(_firmwareDefaultSpeedOfSound);
    set_mode_auto(_firmwareDefaultAutoMode);
    set_ping_interval(_firmwareDefaultPingInterval);
    set_gain_setting(_firmwareDefaultGainSetting);
    pingEnable(_firmwareDefaultPingEnable);
    resetSensorLocalVariables();
}

void Tsr1000::printSensorInformation() const
{
    qCDebug(PING_PROTOCOL_TSR1000) << "TSR1000 Status:";
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- board_voltage:" << _board_voltage;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- pcb_temperature:" << _pcb_temperature;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- processor_temperature:" << _processor_temperature;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- ping_enable:" << _ping_enable;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- distance:" << _distance;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- confidence:" << _confidence;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- transmit_duration:" << _transmit_duration;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- ping_number:" << _ping_number;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- start_mm:" << _scan_start;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- length_mm:" << _scan_length;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- gain_setting:" << _gain_setting;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- mode_auto:" << _mode_auto;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- ping_interval:" << _ping_interval;
    qCDebug(PING_PROTOCOL_TSR1000) << "\t- points:" << QByteArray((const char*)_points.data(), _num_points).toHex(',');
}

void Tsr1000::checkNewFirmwareInGitHubPayload(const QJsonDocument& jsonDocument)
{
    float lastVersionAvailable = 0.0;

    auto filesPayload = jsonDocument.array();
    for (const QJsonValue& filePayload : filesPayload) {
        qCDebug(PING_PROTOCOL_TSR1000) << filePayload["name"].toString();

        // Get version from Tsr1000(\d|)[_|-]V(major).(patch)*.hex where (major).(patch) is <version>
        static const QRegularExpression versionRegex(QStringLiteral(R"(Tsr1000(\d|)[_|-]V(?<version>\d+\.\d+).*\.hex)"));
        auto filePayloadVersion = versionRegex.match(filePayload["name"].toString()).captured("version").toFloat();
        if (filePayloadVersion <= 0) {
            qCWarning(PING_PROTOCOL_TSR1000) << "Invalid version:" << filePayload["name"].toString();
            continue;
        }
        _firmwares[filePayload["name"].toString()] = filePayload["download_url"].toString();

        if (filePayloadVersion > lastVersionAvailable) {
            lastVersionAvailable = filePayloadVersion;
        }
    }
    emit firmwaresAvailableChanged();

    auto sensorVersion = QString("%1.%2")
                             .arg(_commonVariables.deviceInformation.firmware_version_major)
                             .arg(_commonVariables.deviceInformation.firmware_version_minor)
                             .toFloat();
    static QString firmwareUpdateSteps {"https://github.com/bluerobotics/ping-viewer/wiki/firmware-update"};
    if (lastVersionAvailable > sensorVersion) {
        QString newVersionText = QStringLiteral("Firmware update for Ping available: %1<br>").arg(lastVersionAvailable)
            + QStringLiteral("<a href=\"%1\">Check firmware update steps here!</a>").arg(firmwareUpdateSteps);
        NotificationManager::self()->create(newVersionText, "green", StyleManager::infoIcon());
    }
}

void Tsr1000::resetSensorLocalVariables()
{
    _commonVariables.reset();

    _distance = 0;
    _confidence = 0;
    _transmit_duration = 0;
    _ping_number = 0;
    _scan_start = 0;
    _scan_length = 0;
    _gain_setting = 0;
    _speed_of_sound = 0;

    _processor_temperature = 0;
    _pcb_temperature = 0;
    _board_voltage = 0;

    _ping_enable = false;
    _mode_auto = 0;
    _ping_interval = 0;

    _lastPingConfigurationSrcId = -1;
}

Tsr1000::~Tsr1000() { updatePingConfigurationSettings(); }

QDebug operator<<(QDebug d, const Tsr1000::messageStatus& other)
{
    return d << "waiting: " << other.waiting << ", ack: " << other.ack << ", nack: " << other.nack;
}
