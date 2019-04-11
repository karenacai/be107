
try:
    import gopigo as go
    gopigoversion = 2
except ImportError:
    import easygopigo3 as easy
    gopigoversion = 3
if(gopigoversion==3):
    gpg = easy.EasyGoPiGo3()
    def set_right_speed(speed=100):
        gpg.set_speed(speed*3)
    def set_left_speed(speed=100):
        gpg.set_speed(speed*3)
    def enable_encoders():
        pass
    def fwd(dist):
        gpg.drive_cm(dist)
    def bwd(dist):
        gpg.drive_cm(-dist)
    def turn_left(dist):
        StartPositionRight = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
        gpg.set_motor_position(gpg.MOTOR_RIGHT, (StartPositionRight \
                                                + dist))
    def turn_right(dist):
        StartPositionLeft = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
        gpg.set_motor_position(gpg.MOTOR_LEFT, (StartPositionLeft \
                                                + dist))
    def tank_left(degrees):
        gpg.turn_degrees(degrees*5)
    def tank_right(degrees):
        gpg.turn_degrees(-degrees*5)
else:
    def set_right_speed(speed=100):
        go.set_right_speed(speed)
    def set_left_speed(speed=100):
        go.set_left_speed(speed)
    def enable_encoders():
        go.enable_encoders()
    def fwd(dist):
        go.fwd(dist)
    def bwd(dist):
        go.bwd(dist)
    def turn_left(dist):
        go.turn_left(dist)
    def tank_left(degrees):
        go.enc_tgt(0,1,dist)
        go.left_rot()
    def tank_right(degrees):
        go.enc_tgt(0,1,dist)
        go.right_rot()
