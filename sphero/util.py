
def host_to_device_angle(host_angle):
    """
    Transform from euclidean angle to device angle
    :param host_angle: angle degrees in host format
    :type host_angle: int or float
    :return: equal angle in device coordinates
    :rtype: float or int
    """
    return (450.0 - host_angle) % 360


def device_to_host_angle(device_angle):
    """
    Transform from Sphero coordinates to euclidean coordinates
    :param device_angle: angle degrees in device format
    :type device_angle: int or float
    :return: equal angle in host coordinates
    :rtype: float or int
    """
    return (450.0 - device_angle) % 360