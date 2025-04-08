#include "MCAnemometer.h"

Anemometer::Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distance)
{
    loadvalues(ntrig, necho, etrig, eecho, strig, secho,  wtrig,  wecho, distance, distance);
};

Anemometer::Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distancenosu, unsigned int distanceweea)
{
    loadvalues(ntrig, necho, etrig, eecho, strig, secho,  wtrig,  wecho, distancenosu, distanceweea);
};
   

void Anemometer::loadvalues(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distancenosu, unsigned int distanceweea)
{
    this->pinout.n.trig = ntrig;
    pinMode(ntrig, OUTPUT);
    this->pinout.n.echo = necho;
    pinMode(necho, INPUT);
    
    this->pinout.e.trig = etrig;
    pinMode(etrig, OUTPUT);
    this->pinout.n.echo = eecho;
    pinMode(eecho, INPUT);

    this->pinout.s.trig = strig;
    pinMode(strig, OUTPUT);
    this->pinout.s.echo = secho;
    pinMode(secho, INPUT);

    this->pinout.s.trig = wtrig;
    pinMode(wtrig, OUTPUT);
    this->pinout.s.echo = wecho;
    pinMode(wecho, INPUT);


    this->t_offset.nosu=0;
    this->t_offset.wees=0;

    this->dist.nosu = distancenosu;
    this->dist.wees = distanceweea;

};


void Anemometer::readstate()
{
    std::vector<float> n, e, w, s, gusts;

    for(int i=0;i<100;i++)
    {
        n.push_back(reeddistance(pinout.n));
        e.push_back(reeddistance(pinout.e));
        w.push_back(reeddistance(pinout.s));
        s.push_back(reeddistance(pinout.w));
    }

    this->state.wpd.n = std::accumulate(n.begin(), n.end(), 0) / n.size();
    this->state.wpd.e = std::accumulate(e.begin(), e.end(), 0) / e.size();
    this->state.wpd.s = std::accumulate(s.begin(), s.end(), 0) / w.size();
    this->state.wpd.w = std::accumulate(w.begin(), w.end(), 0) / s.size();

    float nosu_s = ((this->state.wpd.n)-(this->state.wpd.s)+this->t_offset.nosu) / 1e6;
    float wees_s = ((this->state.wpd.w)-(this->state.wpd.e)+this->t_offset.wees) / 1e6;

    float windspeed_nosu = this->dist.nosu / nosu_s;
    float windspeed_wees = this->dist.wees / nosu_s;

    state.speed = std::sqrt(nosu_s * windspeed_nosu + nosu_s * windspeed_wees);


    for(int i = 0; i<n.size();i++)
    {
        float nosu = n[i]-s[i]+this->t_offset.nosu;
        float wees = w[i]-e[i]+this->t_offset.wees;
        gusts.push_back(std::sqrt(nosu * nosu + wees * wees));
    }
    this->state.gustswind = *std::max_element(gusts.begin(), gusts.end());
};

float Anemometer::getspeed()
{
    return this->state.speed;
};

float Anemometer::getgustswind()
{
    return this->state.gustswind;
};

float Anemometer::getangle()
{
    float nosu_s = ((this->state.wpd.n)-(this->state.wpd.s)+this->t_offset.nosu) / 1e6;
    float wees_s = ((this->state.wpd.w)-(this->state.wpd.e)+this->t_offset.wees) / 1e6;

    float windspeed_nosu = this->dist.nosu / nosu_s;
    float windspeed_wees = this->dist.wees / wees_s;

    if(nosu_s == 0 || wees_s == 0)
    {
        return 0.0f;
    }

    float angle_rad = std::atan(windspeed_wees / windspeed_nosu);
    float angle_deg = angle_rad * (180/(PI));

    if(windspeed_wees > 0 && windspeed_nosu > 0)
    {
        return angle_deg;
    }
    else if(windspeed_wees > 0 && windspeed_nosu < 0)
    {
        return angle_deg + 90;
    }
    else if(windspeed_wees < 0 && windspeed_nosu < 0)
    {
        return angle_deg + 180;
    }
    else if(windspeed_wees < 0 && windspeed_nosu > 0)
    {
        return angle_deg + 180;
    }
    else
    {
        return -1;
    }
};

void Anemometer::calibrate()
{
    std::vector<float> n;
    std::vector<float> e;
    std::vector<float> w;
    std::vector<float> s;

    for(int i=0;i<100;i++)
    {
        n.push_back(reeddistance(pinout.n));
        e.push_back(reeddistance(pinout.e));
        w.push_back(reeddistance(pinout.s));
        s.push_back(reeddistance(pinout.w));
    }

    this->state.wpd.n = std::accumulate(n.begin(), n.end(), 0) / n.size();
    this->state.wpd.e = std::accumulate(e.begin(), e.end(), 0) / e.size();
    this->state.wpd.s = std::accumulate(s.begin(), s.end(), 0) / w.size();
    this->state.wpd.w = std::accumulate(w.begin(), w.end(), 0) / s.size();

    this->t_offset.nosu = (this->state.wpd.n)-(this->state.wpd.s);
    this->t_offset.wees = (this->state.wpd.e)-(this->state.wpd.w);


};

float Anemometer::reeddistance(uout &comp)
{
    digitalWrite(comp.trig, LOW);
    delayMicroseconds(2000);
    digitalWrite(comp.trig,HIGH);
    delayMicroseconds(10);        
    digitalWrite(comp.trig, LOW);

    
    return pulseIn(comp.echo, HIGH);
};
