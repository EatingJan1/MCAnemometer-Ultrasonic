#include "MCAnemometer.h"

Anemometer::Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, float distance)
{
    loadvalues(ntrig, necho, etrig, eecho, strig, secho,  wtrig,  wecho, distance, distance);
};

Anemometer::Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, float distancenoso, float distanceweea)
{
    loadvalues(ntrig, necho, etrig, eecho, strig, secho,  wtrig,  wecho, distancenoso, distanceweea);
};
   

void Anemometer::loadvalues(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, float distancenoso, float distanceweea)
{
    
    this->pinout.n.trig = ntrig;
    pinMode(ntrig, OUTPUT);
    this->pinout.n.echo = necho;
    pinMode(necho, INPUT);
    
    this->pinout.e.trig = etrig;
    pinMode(etrig, OUTPUT);
    this->pinout.e.echo = eecho;
    pinMode(eecho, INPUT);

    this->pinout.s.trig = strig;
    pinMode(strig, OUTPUT);
    this->pinout.s.echo = secho;
    pinMode(secho, INPUT);

    this->pinout.w.trig = wtrig;
    pinMode(wtrig, OUTPUT);
    this->pinout.w.echo = wecho;
    pinMode(wecho, INPUT);

    
    this->t_offset.nosu=0;
    this->t_offset.wees=0;

    this->dist.nosu = distancenoso;
    this->dist.wees = distanceweea;


    readstate();
};


void Anemometer::readstate()
{
    std::vector<float> n, e, w, s, gusts;

    for (int i = 0; i < 1; i++)
    {
        n.push_back(reeddistance(pinout.n));
        e.push_back(reeddistance(pinout.e));
        w.push_back(reeddistance(pinout.w));
        s.push_back(reeddistance(pinout.s));
    }
    

    this->state.wpd.n = std::accumulate(n.begin(), n.end(), 0.0f) / n.size();
    this->state.wpd.e = std::accumulate(e.begin(), e.end(), 0.0f) / e.size();
    this->state.wpd.w = std::accumulate(w.begin(), w.end(), 0.0f) / w.size();
    this->state.wpd.s = std::accumulate(s.begin(), s.end(), 0.0f) / s.size();
    
    

    float offset_noso = this->t_offset.nosu / 1000.0f;
    float offset_wees = this->t_offset.wees / 1000.0f;
    

    float t_SN = this->state.wpd.s / 1000.0f + offset_noso;
    float t_NS = this->state.wpd.n / 1000.0f + offset_noso;
    
    float windspeed_noso = 0.5f * this->dist.nosu * (1.0f / t_SN - 1.0f / t_NS);
    
    float t_OW = this->state.wpd.e / 1000.0f + offset_wees;
    float t_WO = this->state.wpd.w / 1000.0f + offset_wees;
    
    float windspeed_wees = 0.5f * this->dist.wees * (1.0f / t_OW - 1.0f / t_WO);
    
    
    this->state.speed = std::sqrt(windspeed_noso * windspeed_noso + windspeed_wees * windspeed_wees);

    for(int i = 0; i<n.size();i++)
    {
        float noso = n[i]-s[i]+this->t_offset.nosu;
        float wees = w[i]-e[i]+this->t_offset.wees;
        gusts.push_back(std::sqrt(noso * noso + wees * wees));
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
    // Offset in Sekunden umrechnen (Millisekunden / 1000)
    float offset_noso = this->t_offset.nosu / 1000.0f;
    float offset_wees = this->t_offset.wees / 1000.0f;

    // Berechnungen der Zeiten mit Offset:
    float t_SN = (this->state.wpd.s / 1000.0f) + offset_noso;  // Gegenrichtung
    float t_NS = (this->state.wpd.n / 1000.0f) + offset_noso;  // Mit Wind

    float t_OW = (this->state.wpd.e / 1000.0f) + offset_wees;  // Gegenrichtung
    float t_WO = (this->state.wpd.w / 1000.0f) + offset_wees;  // Mit Wind

    // Wenn eine der Zeitwerte 0 ist (z. B. keine Windmessung), Rückgabe 0
    if (t_SN == 0 || t_NS == 0 || t_OW == 0 || t_WO == 0) {
        return 0.0f;
    }

    // Berechnung der Windgeschwindigkeiten
    float windspeed_noso = 0.5f * this->dist.nosu * (1.0f / t_SN - 1.0f / t_NS);
    float windspeed_wees = 0.5f * this->dist.wees * (1.0f / t_OW - 1.0f / t_WO);

    // Wenn keine Windgeschwindigkeit vorhanden, Rückgabe von 90 oder 270 Grad, je nach Richtung
    if (windspeed_noso == 0) {
        return (windspeed_wees > 0) ? 90.0f : (windspeed_wees < 0 ? 270.0f : 0.0f);
    }

    // Berechnung des Winkels der Windrichtung mit `atan2` (führt die richtige Berechnung durch)
    float angle_rad = std::atan2(windspeed_wees, windspeed_noso);
    float angle_deg = angle_rad * (180.0f / PI);  // Umrechnung von Bogenmaß in Grad

    // Die Richtung des Windes je nach Vorzeichen der Windgeschwindigkeiten anpassen
    if (windspeed_wees > 0 && windspeed_noso > 0) {
        return angle_deg;
    } else if (windspeed_wees > 0 && windspeed_noso < 0) {
        return angle_deg + 90;
    } else if (windspeed_wees < 0 && windspeed_noso < 0) {
        return angle_deg + 180;
    } else if (windspeed_wees < 0 && windspeed_noso > 0) {
        return angle_deg + 180;
    } else {
        return -1;  // Sollte eigentlich nie erreicht werden
    }

};

void Anemometer::calibrate()
{
    std::vector<float> n;
    std::vector<float> e;
    std::vector<float> w;
    std::vector<float> s;

    for(int i=0;i<1;i++)
    {
        n.push_back(reeddistance(pinout.n));
        e.push_back(reeddistance(pinout.e));
        w.push_back(reeddistance(pinout.w));
        s.push_back(reeddistance(pinout.s));
    }

    this->state.wpd.n = std::accumulate(n.begin(), n.end(), 0) / n.size();
    this->state.wpd.e = std::accumulate(e.begin(), e.end(), 0) / e.size();
    this->state.wpd.s = std::accumulate(s.begin(), s.end(), 0) / s.size();
    this->state.wpd.w = std::accumulate(w.begin(), w.end(), 0) / w.size();

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

    
    return pulseIn(comp.echo, HIGH, 30000);
};
