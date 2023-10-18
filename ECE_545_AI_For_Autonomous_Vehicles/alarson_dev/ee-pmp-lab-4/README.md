## Lab 3 - Implementing Noisy Motion Model, Sensor Model (Belief Model) and Particle Filter Model

- Arnie Larson, Jordan Fraser, Sanjar Normuradov
- 11/07/2022
- Group 11 (car 33)


### Motion Model

- Generate a motion model to calculate xt, yt, theatat, add some noise 
- Use a guassian noise model, where commands are drawn N(v_ave, sigma_v); N(delta_ave, sigma_delta) 

- How did you initially chose the noise parameter values. A: Initially I wanted to see the results for average errors of about 1% of the param range
- After picking values, how did you tune them?   A:  made angle STDs a little smaller to narrow arc, made velocity noise a little smaller to narrow blur/thickness of prob density
- I ended up with values of Sigmas of: 
    x = 0.05
    y = 0.05
    theta = 0.1
    delta = 0.05
    speed = 0.05

#### Images

- motion_model1, sigmas: (x=.05; y=.05; theta=.1; delta=.1; speed=.1)   SPEED=1; STEERING_ANGLE=0.34; DT=1
- motion_model2, sigmas: (x=.05; y=.05; theta=.1; delta=.1; speed=.05)  SPEED=1; STEERING_ANGLE=0.34; DT=1
- motion_model3, sigmas: (x=.05; y=.05; theta=.1; delta=.1; speed=.05)  SPEED=0.6; STEERING_ANGLE=0.14; DT=3
- motion_model4, sigmas: (x=.05; y=.05; theta=.05; delta=.01; speed=.05)  SPEED=0.6; STEERING_ANGLE=0.14; DT=3


### Sensor Model

Generate a Sensor model that represents the laser beam scan - incorporate a probability density estimation that estimates the probability that the correct range is d give an estimate of r. 

This probability includes:
- probability that the measurement is too short, e.g. due to clutter, non mapped objects, etc, as an exponential distribution ending at the real point d.
- the probability that the measurement is random noise, uniformly distributed along the range
- the probability that the measurement is long, e.g. reflection and did not return, providing a max range measurement, measured as an impulse distribution at the max point.  
- the probability that the measurement is accurate - as a guassian.

The sensor model is a mixture of these distributions, with weight factors (the z's listed in the code and book)

#### Tuning 

I started with a mixture of about 80% zhit (correct) and sigma of about 3 pixels.  I assumed that errors of around a pixel would probably be unlikely, so I assume that this is low.  If I make it large, on the order of 20 pixels - then a lot of prob mass gets distrubuted over the entire map.  So something between 1 and 10 is probably correct.  But I don't yet know how to tune this.  I would add - since correctly performing lidar will likely have an uncertainty of much less than 1 pixel, there is no reason to assume that sigma hit will be large.

I needed to downsample somewhat aggressively.  From the 500000 pixel points initially generated, I downsampled by a factor of about 10.
For the lidar data I downsampled by a factor of between 4 and 10

After finding some bugs I began tuning the model.  I used a squish factor of ~ .1 to bring in more data.  I down sampled the valid map space randomly so that different runs could potentially show me different results.   I still have not reproduced the pngs particularly well, but the initial one validates that the model appears to be working with no major transformation error type bugs.

Generally - tuning to a few scan bag images seems fairly hard - as it's difficult to impossible to interpret the image weights vs. the actual utility or efficafcy of the weights.  The scaling factor INV_SQUASH_FACTOR plays into which particles are visualized in the demo, but it's unclear how that parameter will play into a real particle filter.

Also - tuning when the vizualization is not completely straightforward to analyze, and there are several dimensions to vary, none of which has completely predictable effects on any metric, is kind of generically hard.


### Resampling

In this portion we are to generate a resampling of a weighted set of hypothesis (the particles) with two methologies.  The naive one yields more variance, and the "low_variance" method yields a better approximation to the true PDF with less trials and visually less variance.

I would say that for the low variance model - with 100 particles, the distribution is very close by 250 trials.
However for the naive model with 100 particles, it takes more like 2500 trials to get an equivalently close good estimate of the pdf.

### Particle Filter

While completing the particle filter I uncovered many oddities in my implementation and went back and tuned the motion model and the sensor model extensively.   I found that if I initially got on the right track my models usually tracked the real bags pretty well.  Lots of obvious symmetry and corridor similarties and some other oddities.  

One specific oddity is that for the full_2x bag I would often get started initially in the wrong direction.  I added a lot of kinematic noise to try to enable my filter to refind the correct location if initially it couldn't get it.  But then my pose would also end up having a higher variance, and being jerky.  

I still am gaining intuition for how the parameters interact with the overall algorithm.

