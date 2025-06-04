#ifndef ANEMOMETER_H
#define ANEMOMETER_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <Arduino.h>

typedef struct {
    unsigned int trig;
    unsigned int echo;
}uout;

class Anemometer
{
    public:
    Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, float distance);
    Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, float distancenosu, float distanceweea);
    
    
    void readstate();
    
    void calibrate();
    
    float getspeed();
    float getangle();
    float getgustswind();
    
    private:
    struct
    {
        uout n;
        uout e;
        uout s;
        uout w;
    }pinout;
    
    struct
    {
        float nosu;
        float wees;
        
    }t_offset;

    struct
    {
        struct
        {
            float n;
            float e;
            float s;
            float w;
        }wpd;

        float speed;
        unsigned int winddirection;
        float gustswind;
    }state;

    struct
    {
        float nosu;
        float wees;
    }dist;
    
    void loadvalues(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, float distancenosu, float distanceweea);
    float reeddistance(uout &comp);
};
#endif