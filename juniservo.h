#ifndef JUNISERVO_H
#define JUNISERVO_H

#include <QSerialPort>
#include <QtDebug>
#include <QThread>

enum STATUS_FLAG{
    SIGNAL_ADDRESS_ERROR=1,
    SIGNAL_CHECKSUM_ERROR=2,
    SIGNAL_INTERVAL_ERROR=4,
    SIGNAL_LIMIT_ERROR=8,
    SIGNAL_FORMAT_ERROR=16,
    SIGNAL_SEND_ERROR=32,
    POSITION_MIN_OVER=1,
    POSITION_MAX_OVER=2,
    TEMPER_OVER=4,
    TORQUE_OVER=8,
    VOLTAGE_MIN_OVER=16,
    VOLTAGE_MAX_OVER=32
};

class JuniServo
{
public:
    JuniServo();
    ~JuniServo();
    bool connect(QString Port, QString Baud);
    void disconnect();

    uint readProdNo(uint id);
    uint readProdVer(uint id);
    uint readFirmVer(uint id);
    uint readPositionSlope(uint id);
    uint readPositionMax(uint id);
    uint readPositionMin(uint id);
    uint readVelocityMax(uint id);
    uint readTorqueMax(uint id);
    uint readVoltageMax(uint id);
    uint readVoltageMin(uint id);
    uint readTemperMax(uint id);
    uint readPosition(uint id);
    int readVelocity(uint id);
    int readTorque(uint id);

    void writeNewTorque(uint id, uint NewTorque);
    void writeNewVelocity(uint id, uint NewVelocity);
    void writeNewPosition(uint id, uint newPosition);
    void writeGroupNewPosition(uint newPosition);
    void writeGroupNewVelocity(uint newVelocity);
    void writeGroupNewTorque(uint newTorque);
    void writeGroupNewTorque(uint newTorque1, uint newTorque2, uint newTorque3);
    void writeGroupNewVelocity(uint newVelocity1, uint newVelocity2, uint newVelocity3);
private:
    QSerialPort *serial;

    QString name = "COM5";
    int32_t baud = 115200;
    QSerialPort::DataBits bit = QSerialPort::DataBits::Data8;
    QSerialPort::Parity parity = QSerialPort::NoParity;
    QSerialPort::StopBits stop = QSerialPort::OneStop;
    QSerialPort::FlowControl flow = QSerialPort::NoFlowControl;
    uint delay = 8;
    uint pre_pos;
    int pre_vel;
    int pre_tor;

    const uint REG_PRODUCT_NO = 0x00;           // Product ID
    const uint REG_PRODUCT_VER = 0x02;          // Product Version
    const uint REG_FIRMWARE_VER = 0x04;         // Product Firmware Version
    const uint REG_SERIAL_NO_SUB = 0x06;        // Product Serial Number Low 2 Byte
    const uint REG_SERIAL_NO_MAIN = 0x08;       // Product Serial Number High 2 Byte

    const uint REG_STATUS_FLAG = 0x0A;          // Current Servo Status
    const uint REG_POSITION = 0x0C;             // Current Position
    const uint REG_VELOCITY = 0x0E;             // Current Velocity
    const uint REG_TORQUE = 0x10;               // Current Torque
    const uint REG_VOLTAGE = 0x12;              // Current Supllied Voltage
    const uint REG_TEMPER = 0x14;               // Current Temperature

    const uint REG_POSITION_NEW = 0x1E;         // New Position Configuration
    const uint REG_VELOCITY_NEW = 0x20;         // New Velocity Configuration
    const uint REG_TORQUE_NEW = 0x22;           // New Torque Configuration
    const uint REG_TURN_NEW = 0x24;             // 360 deg Turn Configuration

    const uint REG_ID = 0x32;                   // ID Configuration
    const uint REG_BAUDRATE = 0x34;             // Baud Rate Configuration
    const uint REG_SIGNAL_MODE = 0x36;          // Signal Tx/Rx Configuration
    const uint REG_SIMPLE_RETURN_DELAY = 0x38;  // Return Packet Transmit Delay Configuration after Receive High Pulse in Simple Mode
    const uint REG_NORMAL_RETURN_DELAY = 0x3A;  // Return Packet Transmit Delay Configuration after Receive High Pulse in Normal Mode
    const uint REG_POWER_CONFIG = 0x46;         // Power Configuration
    const uint REG_EMERGENCY_STOP = 0x48;       // Emergency Stop Configuration(depend on STATUS_FLAG)
    const uint REG_ACTION_MODE = 0x4A;          // Configuration of Acceleration, Deceleration, Velocity, Time and so on
    const uint REG_POSITION_SLOPE = 0x4C;       // Position Slope Configuration
    const uint REG_POSITION_DEADBAND = 0x4E;    // Position Dead Band Configuration
    const uint REG_POSITION_MAX = 0x50;         // Maximum Position Configuration at CW
    const uint REG_POSITION_MIN = 0x52;         // Maximum Position Configuration at CCW
    const uint REG_VELOCITY_MAX = 0x54;         // Maximum Velocity Configuration
    const uint REG_TORQUE_MAX = 0x56;           // Maximum Torque Configuration
    const uint REG_VOLTAGE_MAX = 0x58;          // Maximum Voltage Configuration
    const uint REG_VOLTAGE_MIN = 0x5A;          // Minimum Voltage Configuration
    const uint REG_TEMPER_MAX = 0x5C;           // Maximum Temperature Configuration
    const uint REG_FACTORY_DEFAULT = 0x6E;      // Factory Reset
    const uint REG_CONFIG_SAVE = 0x70;          // Changed Configuration Register Save in Flash Memory
    const uint REG_CONFIG_LOCK = 0x72;          // Locking Change Configuration Register

    void writeData(uint addr, uint id, uint param, uint len = 2);
    uint readDataParsingUint(uint addr, uint id);
    int readDataParsingInt(uint addr, uint id);
};

#endif // JUNISERVO_H
