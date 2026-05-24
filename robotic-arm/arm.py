# Robotic Arm Code



# labrador dog
# class Dog:
#     def __init__(self, name, age):
#         self.name = name
#         self.breed = breed
#         self.age = age

#     def bark(self):
#         print(f"{self.name} is barking")


# "Abstraction"
# Take all the behaviours and properties of an object and we put in a class
# It allows us to hide the complexity of the object and only show the essential features

class Arm:
    def __init__(self, mass, length):
        self.mass = mass
        self.length = length


class RoboticArm:
    def __init__(self, arms):
        self.arms = arms

    def move(self, angle1, angle2, angle3):





        

robotic_arm = RoboticArm([Arm(1, 1), Arm(2, 2), Arm(3, 3)])


# "Encapsulation"
# It allows us to hide the complexity of the object and only show the essential features
robotic_arm.move(1, 2, 3)



# "Modularity"
# Splitting up the code into smaller, defined pieces that all have a specific purpose and then linking them together
# For the robotic arm, the parts or "modules" might be...
# - Arms
# - Joints
# - Motors
# - Sensors
# - Controller
# - Simulation
# - Animation


# "Inheritance"
# Build sub classes from a parent class

class Motor:
    def __init__(self, power):
        self.power = power

    def move(self):

    def apply_torque(self, torque):
        self.power += torque


class DCMotor(Motor):
    def __init__(self, power, voltage):
        super().__init__(power)
        self.voltage = voltage


class StepperMotor(Motor):
    def __init__(self, power, voltage):
        super().__init__(power)
        self.voltage = voltage
        
    def move(self):
        self.power += self.voltage
        
    def apply_torque(self, torque):
        self.power += torque


# "Composition"
# Build objects from other objects

class RoboticArm:
    def __init__(self, arms):
        self.arms = [Arm(1, 1), Arm(2, 2), Arm(3, 3)]




