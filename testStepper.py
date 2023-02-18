# The timer frequency should be as high as possible while still allowing long delays as the motor is accelerated from stop. A timer frequency of 1MHz has been used. A maximum motor speed of 300rpm is then equivalent to a delay count of 1,000. It's necessary to have high timer resolution to give smooth acceleration at high speed.
# 
# f = timer frequency (Hz). (1Mhz)
# delta_t = c/f :  Equation 1
# Motor speed ω (rad/sec) at fixed timer count c :
# Motor step angle(1.8deg): alpha
# omega( ω ) = alpha * f / c : Equation 2
#1rmp = 2*pi/60 [rad/sec] = 0.10471975511965977
#1rad/sce = 60/(2*pi) = 30/pi [rad/sec]=9.55rpm.
# 1deg = pi/180
# 1rad = 180/π = 57.3deg. 1rad/sec = 30/π = 9.55rpm.
# C0 = f*sqrt(2*alph/acc)
# C0^2 / f^2 = 2*alph/acc
# acc = 2*alpha*f^2/c0^2
# acc = 2*1.8*pi/180*1000000^2/0.4142^2
import math

# INPUT
distance=1000 #mm
speed = 70 #mm/sec
accel = 120 #mm/sec^2
decel = accel #mm/sec^2

#------ params --------------------
TICK_PRESCALE  = 128
FREQUENCY=84000000.0/TICK_PRESCALE
TICK_FREQ_SQRT_ERROR_COM = FREQUENCY*0.676

X_MICROSTEPPING=1600*2
X_DIST2STEP_20T5MM = 100 #  100mm/rev (=20TH*5MM)
DIST2STEP=X_MICROSTEPPING/X_DIST2STEP_20T5MM
STEP2DIST=2.0*math.pi/X_MICROSTEPPING
alpha=4.0*math.pi/X_MICROSTEPPING
two_alpha = 2.0*alpha

#--------- Calculation -------------

steps = int(math.fabs(distance)*DIST2STEP)
Ta = speed / accel #%[sec]
Tb = speed / decel #%[sec]
Tc = (steps * alpha - 0.5 * accel * Ta * Ta - 0.5 * decel * Tb * Tb) / speed

if Tc>0:
    Ttotal = Ta + Tc + Tb
else :
    Ttotal = Ta + Tb

  # % 2. Calculate Step
  # % convert Time Zone to Stepper Motor Count
  # % Acceleration Zone
  # % n = speed^2/(2*alpha*accel)

totalSteps = steps
Na = math.floor(speed * speed / (two_alpha * accel))
Nacc = math.floor(steps * decel / (accel + decel))

if Na < Nacc:
    Nb = accel / decel * Na
    Nc = steps - (Na + Nb)
else :
    Na = Nacc
    Nb = steps - Nacc
    Nc = 0

Nac = Na + Nc
NNb = Nb
Cn_const_speed = int((TICK_FREQ_SQRT_ERROR_COM * math.sqrt(two_alpha / accel)) * (math.sqrt(Na + 1) - math.sqrt(Na)))
Cn_acc0 = int((TICK_FREQ_SQRT_ERROR_COM * math.sqrt(two_alpha / accel)))
Cn_dec0 = int((TICK_FREQ_SQRT_ERROR_COM * math.sqrt(two_alpha / decel)))

Cn = Cn_acc0

step_count = 0

Cn = Cn_acc0
step_count = 0