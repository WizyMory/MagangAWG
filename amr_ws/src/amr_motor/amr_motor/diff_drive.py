def twist_to_rpm(v, w, wheel_radius, wheel_base):
    v_l = v - (w * wheel_base / 2.0)
    v_r = v + (w * wheel_base / 2.0)

    rpm_l = (v_l / (2 * 3.1416 * wheel_radius)) * 60.0
    rpm_r = (v_r / (2 * 3.1416 * wheel_radius)) * 60.0

    return rpm_l, rpm_r
