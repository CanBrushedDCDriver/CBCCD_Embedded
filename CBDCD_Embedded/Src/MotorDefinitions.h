#ifndef MOTORDEFINITIONS_H_
#define MOTORDEFINITIONS_H_

//#define DRV8801
#define DRV8825

#ifdef DRV8825
#define STEPPER_DIR_CLOCKWHISE 1
#define STEPPER_DIR_ANTICLOCKWHISE 0

typedef enum
	{micro0=0,
	 micro2=1,
	 micro4=2,
	 micro8=3,
	 micro16=4,
	 micro32=5}
tMicrostepsCount;

#endif

#endif /* MOTORDEFINITIONS_H_ */
