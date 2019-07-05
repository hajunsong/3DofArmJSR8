#include "juniservo.h"

JuniServo::JuniServo()
{
    serial = new QSerialPort();
}
JuniServo::~JuniServo()
{
    serial->close();
}

bool JuniServo::connect(QString Port, QString Baud){
    bool stateConnect = false;
    name = Port;
    baud = Baud.toInt();
    serial->setPortName(name);
    serial->setBaudRate(baud);
    serial->setDataBits(bit);
    serial->setParity(parity);
    serial->setStopBits(stop);
    serial->setFlowControl(flow);
    stateConnect = serial->open(QIODevice::ReadWrite);
    if (stateConnect) {
        qDebug() << "Connected to " + name + " : " + QString::number(baud) + ", " +
                    QString::number(bit) + ", " + QString::number(parity) + ", " +
                    QString::number(stop) + ", " + QString::number(flow);
    }
    else {
        qDebug() << "Error " + serial->errorString();
    }
    return stateConnect;
}

void JuniServo::disconnect()
{
    serial->clear();
    serial->close();
    qDebug() << "Disconnected to " + name;
}

uint JuniServo::readProdNo(uint id)
{
    uint txAddr = REG_PRODUCT_NO;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readProdVer(uint id)
{
    uint txAddr = REG_PRODUCT_VER;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readFirmVer(uint id)
{
    uint txAddr = REG_FIRMWARE_VER;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readPositionSlope(uint id)
{
    uint txAddr = REG_POSITION_SLOPE;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readPositionMax(uint id)
{
    uint txAddr = REG_POSITION_MAX;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readPositionMin(uint id)
{
    uint txAddr = REG_POSITION_MIN;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readVelocityMax(uint id)
{
    uint txAddr = REG_VELOCITY_MAX;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readTorqueMax(uint id)
{
    uint txAddr = REG_TORQUE_MAX;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readVoltageMax(uint id)
{
    uint txAddr = REG_VOLTAGE_MAX;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readVoltageMin(uint id)
{
    uint txAddr = REG_VOLTAGE_MIN;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readTemperMax(uint id)
{
    uint txAddr = REG_TEMPER_MAX;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    return rxParam;
}

uint JuniServo::readPosition(uint id)
{
    uint txAddr = REG_POSITION;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    uint rxParam = readDataParsingUint(txAddr, id);

    if (rxParam == 99999){
        rxParam = pre_pos;
    }
    pre_pos = rxParam;

    return rxParam;
}

int JuniServo::readVelocity(uint id)
{
    uint txAddr = REG_VELOCITY;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    int rxParam = readDataParsingInt(txAddr, id);

    if (rxParam == 99999){
        rxParam = pre_vel;
    }
    pre_vel = rxParam;

    return rxParam;
}

int JuniServo::readTorque(uint id)
{
    uint txAddr = REG_TORQUE;
    writeData(txAddr, id, 0, 0);
    QThread::msleep(delay);
    serial->sendBreak(1);

    serial->waitForReadyRead(-1);
    int rxParam = readDataParsingInt(txAddr, id);

    if (rxParam == 99999){
        rxParam = pre_tor;
    }
    pre_tor = rxParam;

    return rxParam;
}

void JuniServo::writeNewTorque(uint id, uint arg)
{
    writeData(REG_TORQUE_NEW, id, arg);
}

void JuniServo::writeNewVelocity(uint id, uint arg)
{
    writeData(REG_VELOCITY_NEW, id, arg);
}

void JuniServo::writeNewPosition(uint id, uint arg)
{
    writeData(REG_POSITION_NEW, id, arg);
}

void JuniServo::writeGroupNewPosition(uint param)
{
    uint head = 0x96;
    uint id = 0xFF;
    uint addr = REG_POSITION_NEW;
    uchar params[9];
    params[0] = 0x01;
    params[1] = static_cast<uchar>(param%256);
    params[2] = static_cast<uchar>(param/256);
    params[3] = 0x02;
    params[4] = static_cast<uchar>(param%256);
    params[5] = static_cast<uchar>(param/256);
    params[6] = 0x03;
    params[7] = static_cast<uchar>(param%256);
    params[8] = static_cast<uchar>(param/256);
    uint checksum = 0;
    for(uint i = 0; i < 9; i++){
        checksum += params[i];
    }
    uint len = 9;
    checksum += (addr + id + len);
    checksum %= 256;

    QByteArray data;
    data.append(static_cast<char>(head));
    data.append(static_cast<char>(id));
    data.append(static_cast<char>(addr));
    data.append(static_cast<char>(len));
    for(uint i = 0; i < 9; i++){
        data.append(static_cast<char>(params[i]));
    }
    data.append(static_cast<char>(checksum));
    serial->write(data);
    serial->waitForBytesWritten(-1);
}

void JuniServo::writeGroupNewPosition(uint param1, uint param2, uint param3)
{
    uint head = 0x96;
    uint id = 0xFF;
    uint addr = REG_POSITION_NEW;
    uchar params[9];
    params[0] = 0x01;
    params[1] = static_cast<uchar>(param1%256);
    params[2] = static_cast<uchar>(param1/256);
    params[3] = 0x02;
    params[4] = static_cast<uchar>(param2%256);
    params[5] = static_cast<uchar>(param2/256);
    params[6] = 0x03;
    params[7] = static_cast<uchar>(param3%256);
    params[8] = static_cast<uchar>(param3/256);
    uint checksum = 0;
    for(uint i = 0; i < 9; i++){
        checksum += params[i];
    }
    uint len = 9;
    checksum += (addr + id + len);
    checksum %= 256;

    QByteArray data;
    data.append(static_cast<char>(head));
    data.append(static_cast<char>(id));
    data.append(static_cast<char>(addr));
    data.append(static_cast<char>(len));
    for(uint i = 0; i < 9; i++){
        data.append(static_cast<char>(params[i]));
    }
    data.append(static_cast<char>(checksum));
    serial->write(data);
    serial->waitForBytesWritten(-1);
}

void JuniServo::writeGroupNewVelocity(uint param)
{
    uint head = 0x96;
    uint id = 0xFF;
    uint addr = REG_VELOCITY_NEW;
    uchar params[9];
    params[0] = 0x01;
    params[1] = static_cast<uchar>(param%256);
    params[2] = static_cast<uchar>(param/256);
    params[3] = 0x02;
    params[4] = static_cast<uchar>(param%256);
    params[5] = static_cast<uchar>(param/256);
    params[6] = 0x03;
    params[7] = static_cast<uchar>(param%256);
    params[8] = static_cast<uchar>(param/256);
    uint checksum = 0;
    for(uint i = 0; i < 9; i++){
        checksum += params[i];
    }
    uint len = 9;
    checksum += (addr + id + len);
    checksum %= 256;

    QByteArray data;
    data.append(static_cast<char>(head));
    data.append(static_cast<char>(id));
    data.append(static_cast<char>(addr));
    data.append(static_cast<char>(len));
    for(uint i = 0; i < 9; i++){
        data.append(static_cast<char>(params[i]));
    }
    data.append(static_cast<char>(checksum));
    serial->write(data);
    serial->waitForBytesWritten(-1);
}

void JuniServo::writeGroupNewVelocity(uint param1, uint param2, uint param3)
{
    uint head = 0x96;
    uint id = 0xFF;
    uint addr = REG_VELOCITY_NEW;
    uchar params[9];
    params[0] = 0x01;
    params[1] = static_cast<uchar>(param1%256);
    params[2] = static_cast<uchar>(param1/256);
    params[3] = 0x02;
    params[4] = static_cast<uchar>(param2%256);
    params[5] = static_cast<uchar>(param2/256);
    params[6] = 0x03;
    params[7] = static_cast<uchar>(param3%256);
    params[8] = static_cast<uchar>(param3/256);
    uint checksum = 0;
    for(uint i = 0; i < 9; i++){
        checksum += params[i];
    }
    uint len = 9;
    checksum += (addr + id + len);
    checksum %= 256;

    QByteArray data;
    data.append(static_cast<char>(head));
    data.append(static_cast<char>(id));
    data.append(static_cast<char>(addr));
    data.append(static_cast<char>(len));
    for(uint i = 0; i < 9; i++){
        data.append(static_cast<char>(params[i]));
    }
    data.append(static_cast<char>(checksum));
    serial->write(data);
    serial->waitForBytesWritten(-1);
}

void JuniServo::writeGroupNewTorque(uint param)
{
    uint head = 0x96;
    uint id = 0xFF;
    uint addr = REG_TORQUE_NEW;
    uchar params[9];
    params[0] = 0x01;
    params[1] = static_cast<uchar>(param%256);
    params[2] = static_cast<uchar>(param/256);
    params[3] = 0x02;
    params[4] = static_cast<uchar>(param%256);
    params[5] = static_cast<uchar>(param/256);
    params[6] = 0x03;
    params[7] = static_cast<uchar>(param%256);
    params[8] = static_cast<uchar>(param/256);
    uint checksum = 0;
    for(uint i = 0; i < 9; i++){
        checksum += params[i];
    }
    uint len = 9;
    checksum += (addr + id + len);
    checksum %= 256;

    QByteArray data;
    data.append(static_cast<char>(head));
    data.append(static_cast<char>(id));
    data.append(static_cast<char>(addr));
    data.append(static_cast<char>(len));
    for(uint i = 0; i < 9; i++){
        data.append(static_cast<char>(params[i]));
    }
    data.append(static_cast<char>(checksum));
    serial->write(data);
    serial->waitForBytesWritten(-1);
}

void JuniServo::writeGroupNewTorque(uint param1, uint param2, uint param3)
{
    uint head = 0x96;
    uint id = 0xFF;
    uint addr = REG_TORQUE_NEW;
    uchar params[9];
    params[0] = 0x01;
    params[1] = static_cast<uchar>(param1%256);
    params[2] = static_cast<uchar>(param1/256);
    params[3] = 0x02;
    params[4] = static_cast<uchar>(param2%256);
    params[5] = static_cast<uchar>(param2/256);
    params[6] = 0x03;
    params[7] = static_cast<uchar>(param3%256);
    params[8] = static_cast<uchar>(param3/256);
    uint checksum = 0;
    for(uint i = 0; i < 9; i++){
        checksum += params[i];
    }
    uint len = 9;
    checksum += (addr + id + len);
    checksum %= 256;

    QByteArray data;
    data.append(static_cast<char>(head));
    data.append(static_cast<char>(id));
    data.append(static_cast<char>(addr));
    data.append(static_cast<char>(len));
    for(uint i = 0; i < 9; i++){
        data.append(static_cast<char>(params[i]));
    }
    data.append(static_cast<char>(checksum));
    serial->write(data);
    serial->waitForBytesWritten(-1);
}

void JuniServo::writeData(uint addr, uint id, uint param, uint len){
    uint txHead = 0x96;
    uint txParams[2] = {0,0}, txChecksum;
    if (len > 0){
        txParams[0] = param%256;
        txParams[1] = param/256;
    }

    QByteArray txData;
    txChecksum = (id + addr + len + txParams[0] + txParams[1])%256;
    txData.append(static_cast<char>(txHead));
    txData.append(static_cast<char>(id));
    txData.append(static_cast<char>(addr));
    txData.append(static_cast<char>(len));
    for (uint i = 0;i < len; i++){
        txData.append(static_cast<char>(txParams[i]));
    }
    txData.append(static_cast<char>(txChecksum));
    serial->write(txData);
    serial->waitForBytesWritten(-1);
}

uint JuniServo::readDataParsingUint(uint addr, uint id){
    QByteArray rxData = serial->readAll();
    char rxHead = 0x69;
    QList<QByteArray> rxDataList = rxData.split(static_cast<char>(rxHead));
    int len = rxDataList.length();
    uint rxParam = 99999;
    for(int i = 0; i < len; i++){
        QByteArray rxDataSplit = rxDataList[i];
        uchar rxAddr = static_cast<uchar>(rxDataSplit[1]);
        if (rxAddr == addr){
            uchar rxID = static_cast<uchar>(rxDataSplit[0]);
            if (rxID == id){
                uint rxBits[2] = {static_cast<uchar>(rxDataSplit[3]), static_cast<uchar>(rxDataSplit[4])};
                rxParam = rxBits[1]*256 + rxBits[0];
                break;
            }
        }
    }
    return rxParam;
}

int JuniServo::readDataParsingInt(uint addr, uint id){
    QByteArray rxData = serial->readAll();
    char rxHead = 0x69;
    QList<QByteArray> rxDataList = rxData.split(static_cast<char>(rxHead));
    int len = rxDataList.length();
    int rxParam = 99999;
    for(int i = 0; i < len; i++){
        QByteArray rxDataSplit = rxDataList[i];
        uchar rxAddr = static_cast<uchar>(rxDataSplit[1]);
        if (rxAddr == addr){
            uchar rxID = static_cast<uchar>(rxDataSplit[0]);
            if (rxID == id){
                uint rxBits[2] = {static_cast<uchar>(rxDataSplit[3]), static_cast<uchar>(rxDataSplit[4])};
                rxParam = static_cast<char>(rxBits[1])*256 + static_cast<char>(rxBits[0]);
                break;
            }
        }
    }
    return rxParam;
}
