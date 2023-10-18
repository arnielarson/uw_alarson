# UWEE-592-Radar

## Arnold Larson

Radar Signals class taken in Autumn 2022 with Matt Reynolds at UW ECE.

## Final Project

Creating a simple radar detector using a continuous wave (CW) radar module.  Very interesting figure out strategies for collecting real time data.  

## To be continued

This will be an interesting module to continue playing with to practice embedded programming concepts and real time data collection strategies.  I also should fully update the SNR and threshold detection algorithm.


## Completed 2022

### Needs to do:

 [*] Create a data sampler, record data for T seconds output a spectrogram

 [*] Add in processing of targets in that window

 [*] Create a data logger, record data for T seconds (wav files)

 [*] Create a data collection strategy

 [*] Create a real time data displayer, update display

 [*] Analyze data, how to pick out strongest signals?

 [*] Create batch program to process data..

 [*] Ideally create a real time program that displays data with Qt -

 [ ] Can I read data over UDP (unecessary, used Python multiprocessing for IPC classes)


 Experimental data gathering.

 [*] Person walkng

 [*] Person running

 [*] Person riding bicycle

 doppler shifts will be FD= FT * (2v/c)
 FT = 10 GHz, per m/s, FD ~ 2 * 1 * 10e9 / 3e8 ~ 60 Hz/ (m/s)
 m/s to mpg -   1m = 3.3 ft, 1 mile = 5280 ft, 1 mile = 1600m, 1 hr * 3600s
 1 m/s * 3600 s/hr / (1600 m/mile) = 2.25 mph

 So 25 mph is about 10 or 11 m/s or F -> 600 Hz to 700Hz
