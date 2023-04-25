//
// Created by Stephanie Georges on 4/25/23.
//

#ifndef EFFECTIVE_TELEGRAM_IR_H
#define EFFECTIVE_TELEGRAM_IR_H

struct IR {
    uint32_t pin;
    uint16_t (*read)(struct Sensor*);
};

void initSensor(struct Sensor*, uint32_t);
uint16_t readSensor(struct Sensor*);

#endif //EFFECTIVE_TELEGRAM_IR_H
