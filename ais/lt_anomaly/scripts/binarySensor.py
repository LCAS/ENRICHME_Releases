from magnitudeSensor import MagnitudeSensor

class BinarySensor(MagnitudeSensor):

    def castValue(self,currVal):

        if isinstance(currVal, unicode):
            if currVal == 'CLOSED':
                currVal = 0
            elif currVal == 'OPEN':
                currVal = 1
            else:
                print("Don't know how to handle ",currVal, " values")
                exit()

        return currVal