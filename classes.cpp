#include<Arduino.h>
class PIDController {
    float KP = 0, KI = 0, KD = 0, Omax = 0, Imax = 0;
    float Iacc = 0,  selisihsebelum = 0;
    float t_, tireset;
    float emin;
    int resetimode;
  public:
    PIDController(float kp, float ki, float kd, float outmax, float imax) {
      KP = kp; KI = ki; KD = kd; Imax = imax; Omax = outmax;
      t_ = 0;
      tireset = 0;
      resetimode = 0;
      emin = 0;
    }
    float calc(float nilai, float target, float dt) {
      float selisih = nilai - target;
      //float Ppart = KP * selisih;
      //if (Ppart > Imax)Ppart = Imax; else if (Ppart < -Imax)Ppart = -Imax;
      //float newIacc = Iacc + selisih * KI * dt;
      t_ += dt;
      if (resetimode) {
        if (selisih * selisihsebelum < 0) {
          Iacc = 0;
        }
        if (tireset > 0) {
          if (t_ > tireset) {
            t_ -= tireset;
            Iacc = 0;
          }
        }
      }
      else {
        Iacc += selisih * KI * dt;
      }
      if (emin > 0) {
        if (fabs(selisih) > emin) {
          Iacc = 0;
        }
      }
      //float Iaccmax = Imax - fabs(Ppart);
      //if (Iaccmax < 0)Iaccmax = 0;
      if (Iacc > Imax)Iacc = Imax; else if (Iacc < -Imax)Iacc = -Imax;

      float diff = (selisih - selisihsebelum) / dt;
      selisihsebelum = selisih;
      //float diff = (nilai - nilaisebelum) / dt;
      //nilaisebelum = nilai;
      //Serial.println(diff*10000);
      float output = KP * selisih + Iacc + KD * diff;
      if (output > Omax)output = Omax; else if (output < -Omax)output = -Omax;
      return output;
    }
    void setKP(float x) {
      KP = x;
    }
    void setKI(float x) {
      KI = x;
    }
    void setKD(float x) {
      KD = x;
    }
    void setIreset(int mode) {
      resetimode = mode;
    }
    void setTIreset(float t) {
      t_ = t;
    }
    void setEminInt(float e) {
      emin = e;
    }
    void setOutmax(float x) {
      Omax = x;
    }
    void setImax(float x) {
      Imax = x;
    }
    float getKP() {
      return KP;
    }
    float getKI() {
      return KI;
    }
    float getKD() {
      return KD;
    }
    void reset() {
      Iacc = 0;
      selisihsebelum = 0;
    }
};
class Smoothfilter {
  private:
    float outval;
    int navg;
  public:
    Smoothfilter(int n) {
      navg = n;
      outval = 0;
    }
    float calc(float x) {
      outval = ((outval * navg) + x) / (navg + 1.);
      return outval;
    }
    float setvalue(float x) {
      outval = x;
    }
    float setnavg(int n) {
      navg = n;
    }
};
class RateLimiter {
  private:
    int target;
    int posisisekarang;
    float r;
    long *timervar;
    long lastt, updateinterval;
  public:
    RateLimiter(float maxrate, long *t) {
      r = maxrate;
      timervar = t;
      posisisekarang = 0;
      updateinterval = 10;
    }
    void jumpto(int posisi) {
      posisisekarang = posisi;
      target = posisi;
    }
    int update(int newtarget) {
      target = newtarget;
      //long tt = millis();
      if ((*timervar - lastt) > updateinterval) {
        float selisih = (*timervar - lastt) * r;
        lastt = *timervar;
        if (target > posisisekarang) {
          if (target - posisisekarang > selisih) {
            posisisekarang += selisih;
          }
          else {
            posisisekarang = target;
          }
        }
        else {
          if (posisisekarang - target > selisih) {
            posisisekarang -= selisih;
          }
          else {
            posisisekarang = target;
          }
        }
        //Serial.println(selisih);
      }
      return posisisekarang;
    }
};
