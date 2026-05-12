
class FirmwareVersion():
    """
    Manage comparision of firmware versions
    """

    def __init__(self, version_str):
        self.device = 'NONE'
        self.major = 0
        self.minor = 0
        self.bugfix = 0
        self.protocol = 0
        self.valid = False
        self.from_string(version_str)

    def __str__(self):
        return self.to_string()

    def to_string(self):
        """
        Version is represented as Stepper.v0.0.1p0 for example
        """
        return self.device + '.v' + str(self.major) + '.' + str(self.minor) + '.' + str(self.bugfix) + 'p' + str(
            self.protocol)

    def __gt__(self, other):
        if not self.valid or not other.valid:
            return False
        if self.protocol > other.protocol:
            return True
        if self.protocol < other.protocol:
            return False
        return (self.major, self.minor, self.bugfix) > (other.major, other.minor, other.bugfix)

    def __lt__(self, other):
        if not self.valid or not other.valid:
            return False
        if self.protocol < other.protocol:
            return True
        if self.protocol > other.protocol:
            return False
        return (self.major, self.minor, self.bugfix) < (other.major, other.minor, other.bugfix)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __eq__(self, other):
        if not other or not self.valid or not other.valid:
            return False
        return self.major == other.major and self.minor == other.minor and self.bugfix == other.bugfix and self.protocol == other.protocol

    def same_device(self, d):
        return d == self.device

    def from_string(self, x):
        # X is of form 'Stepper.v0.0.1p0' or 'hello-stepper2.v0.0.1p0'
        try:
            xl = x.split('.v')
            if len(xl) != 2:
                raise Exception('Invalid version len')
            device = xl[0]
            v_parts = xl[1].split('.')
            if len(v_parts) != 3:
                raise Exception('Invalid version format')
            major = int(v_parts[0])
            minor = int(v_parts[1])
            bugfix = int(v_parts[2][0:v_parts[2].find('p')])
            protocol = int(v_parts[2][(v_parts[2].find('p') + 1):])
            self.device = device
            self.major = major
            self.minor = minor
            self.bugfix = bugfix
            self.protocol = protocol
            self.valid = True
        except(ValueError, Exception):
            pass # print('Invalid version format in tag: %s' % x)
