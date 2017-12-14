
#ifndef _PCA9685_H__
#define _PCA9685_H__

class PCA9685
{
public:
    //- Constructors:
    PCA9685();
	
	//- Functions:
	void init(void);
    void setPWMFreq(int freq);
    void setPWM(int channel, int on, int off);
    void setAllPWM(int on, int off);
    void setPin(int pin, int value);

	void goForward(void);
	void goBack(void);
	void goLeft(void);
	void goRight(void);
	void stop(void);
	void leftSmoothly(void); //new function
	void rightSmoothly(void); //new function

	void setSpeed(int pin, int speed);
	void setNormalSpeed(int speed);
	void setModSpeed(int fr, int bk);
    
    void onBuzz(void);
    void offBuzz(void);
	
	~PCA9685();
private:
    int	fd;
	int nSpeed;
	int frSpeed;
	int bkSpeed;
	
	int enAPin;
	int en1Pin;
	int en2Pin;
	int enBPin;
	int en3Pin;
	int en4Pin;		
    int BuzzPin;
};

#endif // _PCA9685_H__
