#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <string>
#include <chrono>
#include <vector>
#include <bits/stdc++.h> 
#include <cstdlib>
#include <math.h>
using namespace std;


// I2C address
static const uint8_t ACC_ADDR = 0x6B;

// Registers
static const uint8_t REG_CTL_1 = 0x20;
static const uint8_t REG_DATAX0 = 0x2c;

static const int readsize = 2;



/*******************************************************************************
 * Function Declarations
 */
int reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int reg_read(   i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

/*******************************************************************************
 * Function Definitions
 */

// Write 1 byte to the specified register
int reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), true);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, true);

    return num_bytes_read;
}

float getAverage(vector<float> v) {
  if (v.empty()){
    return 0;
  }
  auto const count = static_cast<float>(v.size());
  return reduce(v.begin(), v.end()) / count;
}

vector<float> rejectoutliers(vector<float> v) {
    printf("%i", v.size());
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    float median = v[n];
    vector<float> out;
    for(float i : v){
        if (abs(i - median) < 50){
            out.push_back(i);
        }
    }
    printf("%i", out.size());
    return out;
}



int16_t acc_z;
float acc_z_f = 0.0;

    // Pins
const uint sda_pin = 2;
const uint scl_pin = 3;

// Ports
i2c_inst_t *i2c = i2c1;

// Buffer to store raw reads
uint8_t readbuffer[readsize];

int setupits = 50;

unsigned int iteration = 0;
vector<float> setuperrors;
float z_read_error = 0;
float zrot = 0;
float desiredrot = 0;

float acceptableroterror = 1;
float rotcorrectionspeed = 1000;
float accsensitivity = 10;
int rotovershoot = 8;

int speedbuffer[2] = {0, 0};
////////////////
int debug = 0;
////////////////

int ticktime_ms = 10;

float delta_time = 0;

bool stopped = false;

int state;

//uint MotorSignal[motornum]
const uint MotorSignal[2] = {8, 9};
//const uint 1MotorSignal = 8;
//const uint 2MotorSignal = 9;

//uint MotorDir[motornum][dirpin]
const uint MotorDir[2][2] = {{4, 6}, {7, 5}};
//const uint 1MotorDir1 = 6;
//const uint 1MotorDir2 = 4;
//const uint 2MotorDir1 = 7;
//const uint 2MotorDir2 = 5;

void SetMotorSpeed(int8_t motor /*0-1*/, int inspeed /*-65535 - 65535*/){
    
    bool fwd = inspeed >= 0;
    unsigned short speed = (unsigned short)abs(clamp(inspeed, -65535, 65535));

    //cout << "speed: " << speed << "\n";

    if (debug == 0){
        pwm_set_gpio_level(MotorSignal[motor], speed);
    }else if (debug == 1){
        pwm_set_gpio_level(MotorSignal[motor], 0);
    }

    gpio_put(MotorDir[motor][0], fwd);
    gpio_put(MotorDir[motor][1], !fwd);

}

void adjustforrotation(){

    float deltarot = 360;
    for (int i = -1; i <= 1; i++){
        float tmp = desiredrot - zrot + i*360;
        if (abs(tmp) < abs(deltarot)){
            deltarot = tmp;
        }
    }
    cout << deltarot << "\n";

    if (abs(deltarot) < acceptableroterror){
        return;
    }else {
        cout << desiredrot << "\t" << zrot <<"\n";
        if (abs(deltarot) > acceptableroterror){

            
            //if (abs(deltarot) < rotovershoot){
            if (deltarot > 0){
                deltarot += rotovershoot;
            }else{
                deltarot -= rotovershoot;
            }
            //}

            deltarot -= acc_z_f*accsensitivity;

            speedbuffer[0] += deltarot * rotcorrectionspeed;
            speedbuffer[1] -= deltarot * rotcorrectionspeed;
            cout << speedbuffer[0] << " " << speedbuffer[1] << "\t" << (acc_z_f*accsensitivity) << "\n";
        }
    }

}


void processrotation(){
    for (int i = 0; i < readsize; i++){
        uint8_t rb;
        reg_read(i2c, ACC_ADDR, REG_DATAX0 + i, &rb, 1);
        readbuffer[i] = rb;
    }

    // Convert 2 bytes (little-endian) into 16-bit integer (signed)
    acc_z =  (int16_t)((readbuffer[1] << 8) | readbuffer[0]);

    acc_z_f = ((static_cast<float>(acc_z) * ((float) ticktime_ms/1000.0f)) - z_read_error);

    if (abs(acc_z_f) > 0.01){
        zrot += acc_z_f * 0.01f; //cheeky angle correction, the accelorometer treats a full rotation as 40000 units, instead of 360
        //zrot = remainder(zrot, 360.0f);
    }


    switch(state) {
        case 0:
            setuperrors.push_back(acc_z_f);
            break;
        case 1:
            setuperrors = rejectoutliers(setuperrors);
            z_read_error = getAverage(setuperrors);
            printf("read error: %f", z_read_error);
            break;
        case 2:
            if (iteration % 5 == 0){
                printf("%f\n", zrot);
            }
            if (iteration % 200 == 0){
                desiredrot += 45;
                if (desiredrot > 180) {
                    desiredrot -= 360;
                } 
            }
            adjustforrotation();
    }
    
}

bool loop (struct repeating_timer *t){
    speedbuffer[0] = 0;
    speedbuffer[1] = 0;
    // Read the Z value from registers (16 bits total); readsize = 2

    /*set the state
        0 = setup
        1 = finalise setup
        2 = seek
    */
    if (iteration < setupits){
            state = 0;
        }
    else if(iteration == setupits){
        state = 1;
    }else {
        state = 2;
    }

    processrotation();

    SetMotorSpeed(0, speedbuffer[0]);
    SetMotorSpeed(1, speedbuffer[1]);
    iteration++;
    cout << "tick time: " << ticktime_ms << "\t Z rotation: " << zrot << "\n";
    return true;
}

int main() {
    // Setup code

    stdio_init_all();

    i2c_init(i2c, 400 * 1000);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    //init motor pins
    for (int i = 0; i < 2; i++){

        gpio_set_function(MotorSignal[i], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(MotorSignal[i]);
        printf ("slice_num %i: \t%a");    //COMPILE ERROR STRING MISMATCH SKIDOODLES
    
        pwm_config config = pwm_get_default_config();
        // Set divider, reduces counter clock to sysclock/this value
        pwm_config_set_clkdiv(&config, 4.f);
        // Load the configuration into our PWM slice, and set it running.
        pwm_init(slice_num, &config, true);


        for (int j = 0; j < 2; j++){
            gpio_init(MotorDir[i][j]);
            gpio_set_dir(MotorDir[i][j], GPIO_OUT);        }
    }


    // Read Power Control register
    reg_read(i2c, ACC_ADDR, REG_CTL_1, readbuffer, 1);
    //printf("%02x\r\n", readbuffer[0]);


    readbuffer[0] = 0b00001111;
    reg_write(i2c, ACC_ADDR, REG_CTL_1, &readbuffer[0], 1);


    // Test: read Power Control register back to make sure Measure bit was set
    reg_read(i2c, ACC_ADDR, REG_CTL_1, readbuffer, 1);
    printf("%02x\r\n", readbuffer[0]);

    sleep_ms(100);

    printf("hello\n");
    SetMotorSpeed(0, 0);
    SetMotorSpeed(1, 0);

    struct repeating_timer timer;
    add_repeating_timer_ms(ticktime_ms, loop, NULL, &timer);
    while (!stopped){
        sleep_ms(1000);
    }
}