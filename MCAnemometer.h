#ifndef ANEMOMETER_H
#define ANEMOMETER_H_H

#include <Arduino.h>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

typedef struct {
    unsigned int trig;
    unsigned int echo;
}uout;

class Anemometer
{
    public:
    Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distance);
    Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distancenoso, unsigned int distanceweea);
    
    
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
        float noso;
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
        float noso;
        float wees;
    }dist;
    
    
    
    
    void loadvalues(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distancenoso, unsigned int distanceweea);
    
    float reeddistance(uout &comp);




};

#endif