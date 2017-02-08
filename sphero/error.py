

class SpheroError(Exception):
    pass


class SpheroConnectionError(SpheroError):
    """
    Exception used when there are some connection errors with the device
    """
    pass


class SpheroFatalError(SpheroError):
    """
    Exception used when total failure of sphero
    """


class SpheroRequestError(SpheroError):
    """
    Exception used when a command has failed
    """