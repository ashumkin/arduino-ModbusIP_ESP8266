/*
    ModbusIP_ESP8266.cpp - Source for Modbus IP ESP8266 Library
    Copyright (C) 2015 André Sarmento Barbosa
*/
#include "ModbusIP_ESP8266.h"

#define ModbusPacketLen 7
#define numOfClients 4
WiFiClient clients[numOfClients];

WiFiServer server(MODBUSIP_PORT);

ModbusIP::ModbusIP() {

}

void ModbusIP::config(const char* ssid, const char* password) {
    WiFi.begin(ssid, password);
    server.begin();
}

void ModbusIP::handle(int raw_len, WiFiClient client) {
    for (int j = 0; j < ModbusPacketLen; j++) _MBAP[j] = client.read(); //Get MBAP

    _len = _MBAP[4] << 8 | _MBAP[5];
    _len--; // Do not count with last byte from MBAP
    if (_MBAP[2] != 0 || _MBAP[3] != 0) return;   //Not a MODBUSIP packet
    if (_len > MODBUSIP_MAXFRAME) return;      //Length is over MODBUSIP_MAXFRAME
    _frame = (byte*) malloc(_len);

    raw_len = raw_len - ModbusPacketLen;
    for (int j = 0; j < raw_len; j++)  _frame[j] = client.read(); //Get Modbus PDU

    this->receivePDU(_frame);
    client.flush();
    if (_reply != MB_REPLY_OFF) {
        //MBAP
        _MBAP[4] = (_len + 1) >> 8;     //_len+1 for last byte from MBAP
        _MBAP[5] = (_len + 1) & 0x00FF;

        size_t send_len = (unsigned int)_len + ModbusPacketLen;
        uint8_t sbuf[send_len];

        for (int j = 0; j < ModbusPacketLen; j++)     sbuf[j] = _MBAP[j];
        for (int j = 0; j < _len; j++)  sbuf[j+ModbusPacketLen] = _frame[j];

        client.write(sbuf, send_len);
    }

    free(_frame);
    _len = 0;
}

void ModbusIP::task() {
    int raw_len[numOfClients] = { 0 };
    for (int i = 0; i < numOfClients - 1; i++) {
        if (clients[i]) {
            if (clients[i].status() == CLOSED) {
                clients[i].stop();
                // clients[i] = null;
            }
            if (clients[i].available()) {
                while (clients[i].available() > raw_len[i]) {  //Computes data length
                   raw_len[i] = clients[i].available();
                   delay(1);
                }
            }
        } else {
            clients[i] = server.available();
        }
    }
    for (int i = 0; i < numOfClients - 1; i++) {
        if (clients[i]) {
            if (raw_len[i] > ModbusPacketLen) {
                this->handle(raw_len[i], clients[i]);
            }
        }
    }
}

    /*
    uint8_t buffer[128] = {0};
    uint8_t mux_id;
    uint32_t len = _wifi->recv(&mux_id, buffer, sizeof(buffer), 100);

    if (len > 0) {
        int i = 0;
        while (i < 7) {
            _MBAP[i] = buffer[i];
             i++;
        }

        _len = _MBAP[4] << 8 | _MBAP[5];
        _len--; // Do not count with last byte from MBAP
        if (_MBAP[2] !=0 || _MBAP[3] !=0) return;   //Not a MODBUSIP packet
        if (_len > MODBUSIP_MAXFRAME) return;      //Length is over MODBUSIP_MAXFRAME

        _frame = (byte*) malloc(_len);
        i = 0;
        while (i < _len){
            _frame[i] = buffer[7+i];  //Forget MBAP and take just modbus pdu
            i++;
        }

        this->receivePDU(_frame);

        if (_reply != MB_REPLY_OFF) {
            //MBAP
            _MBAP[4] = _len >> 8;
            _MBAP[5] = _len | 0x00FF;
            buffer[4] = _MBAP[4];
            buffer[5] = _MBAP[5];

            i = 0;
            while (i < _len){
                buffer[i+7] = _frame[i];
                i++;
            }
            _wifi->send(mux_id, buffer, _len + 7);
            _wifi->releaseTCP(mux_id);
        }

        free(_frame);
        _len = 0;
    }

}
*/
