import serial as _serial
import time   as _time

# Serial COM markers
endMarker   = '\n'
terminator  = '\r\n'

_debug_enabled = True 


class Monochromator_api():
    """
    Commands-only object for interacting with the arduino based
    Atomic Spectra Monochromator hardware.
    
    Parameters
    ----------
    port='COM4' : str
        Name of the port to connect to.
        
    baudrate=115200 : int
        Baud rate of the connection. Must match the instrument setting.
        
    timeout = None : int
        How long to wait for responses before giving up (s). 
        
    """
    def __init__(self, port='COM4', baudrate=115200, timeout=None):
                
        if not _serial:
            print('You need to install pyserial to use the Atomic Spectra Monochromator.')
            self.simulation_mode = True
        
        self.simulation_mode = False
        
        # If the port is "Simulation"
        if port=='Simulation': self.simulation_mode = True
        
        # If we have all the libraries, try connecting.
        if not self.simulation_mode:
            try:
                # Create the instrument and ensure the settings are correct.
                self.serial = _serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
                
            # Something went wrong. Go into simulation mode.
            except Exception as e:
                  print('Could not open connection to "'+port+':'+'" at baudrate '+str(baudrate)+'. Entering simulation mode.')
                  print(e)
                  self.simulation_mode = True
                  self.serial = None
                
        # Container for scan data as it is dynamically acquired          
        self.scan_data = ''
                
        # Give the arduino time to run the setup loop
        _time.sleep(2)
    
    def getStatus(self):
        """
        Get the current homing status of the Monochromator.
        """
        self.write('STAT?')
        
        return self.read()

    def getID(self):
        """
        Get the identification string of firware currently on the arduino board.

        Returns
        -------
        str
            A string describing the arduino sketch version and compilation date.

        """
        self.write('*IDN?')
        
        return self.read()
    
    def getDirection(self):
        """
        Get the current motor direction.

        Returns
        -------
        direction: bool
            False and True are the forward and backward directions, respectively.

        """
        self.write("STAGE:DIR?")
        
        return bool(self.read())    
    
    def getPmt(self):
        """
        Get the photomultiplier tube (PMT) voltage.

        Returns
        -------
        int
            Digitized voltage [0-2**(bit_depth)-1].

        """
        
        self.write("PMT:VAL?")
        
        return int(self.read())    
        
    def getPosition(self):
        """
        Get the current absolute position of the Monochromator motor.

        """
        
        self.write('POS?')
        
        s = self.read()
        
        try:     result = int(s)
        except:  result = s
        
        return s
    
    def home(self):
        """
        Home the MonoChromator.
        """
        self.write('HOME')
        
        return self.read()
    
    def getMaxLimit(self):
        return
    
    def getMinLimit(self):
        return
    
    def setDirection(self, direction):
        """
        Set the direction of the motor.
        
        Parameters
        ----------
        direction: bool
            False and True are the forward and backward directions, respectively.
            
        """
        self.write("DIR,%d"%direction)
        
    def setPMTOffset(self, _offset):
        self.write('PMT:OFFSET %d'%(_offset))
        return self.read() == str(_offset)
        
    def setPosition(self,position):
        """
        Set the position of the monochromator.
        
        Parameters
        ----------
        position : int
            Absolute position to which the stage will be moved.

        Returns
        -------
        None.

        """
        self.write('POS %d'%(position)) 
        return self.read()
        
    def goToMax(self):
        return
        
    def goToMin(self):
        return
    
    
    ## Serial COM ##
    
    def write(self,raw_data):
        """
        Writes data to the serial line, formatted appropriately to be read by the monochromator.        
        
        Parameters
        ----------
        raw_data : str
            Raw data string to be sent to the arduino.
        
        Returns
        -------
        None.
        
        """
        encoded_data = (raw_data + endMarker).encode()
        self.serial.write(encoded_data) 
    
    def read(self):
        """
        Reads data from the serial line.
        
        Returns
        -------
        str
            Raw data string read from the serial line.
        """
        return self.serial.read_until(expected = terminator.encode()).decode().strip(terminator)
            
    def disconnect(self):
        """
        Disconnects the port.
        """
        if not self.simulation_mode and self.serial != None: 
            self.serial.close()
            self.serial = None
