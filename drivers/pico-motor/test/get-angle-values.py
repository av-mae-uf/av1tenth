from testSerial import convert_speed_to_angle, map_range

# From the arduino:
# 90 deg gives us 17 as a duty which is a pulse of 1.478 ms
# 95 deg gives us 18 as a duty which is a pulse of 1.564 ms
# 103 deg gives us 18 as a duty which is a pulse of 1.651 ms

for i in range(-30,30,1):
    val = convert_speed_to_angle(i/10.0)
    num = map_range(val, 0, 180, 7, 28)
    pulse = map_range(num, 7, 28, 595, 2425)
    print(f"{i/10.0} m/s, {val} deg, {num}/255 => {pulse} ")





