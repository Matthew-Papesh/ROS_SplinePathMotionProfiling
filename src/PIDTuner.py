from typing import Callable
import rospy
import handler

# coefficient type mappings 
class ktype: # k coeff types
    KP, KI, KD = 0, 1, 2
class etype: # eval test types
    BASE, UPPER, LOWER = 3, 4, 5

class PIDTuner:
    """
    Computes optimal PID coefficients by tuning one coefficient at a time in a specific order. Tuning a given coefficient holds the others constant such that 
    the "current" value for the cofficient is tested, then a larger value and lesser value to then determine which of the three cases has minimal performance error. 
    The case with minimal error is then set to the "current" coefficient value as the process of stepping is repeated; larger/less values are found by takig the 
    average between the base value and the point at (+/-) range from a base value (midpoints between low/high bounds and the base value)
    """
    # number of epochs to test for average pid eval
    eval_epochs = 10

    def __init__(self, base_kp: float, base_ki: float, base_kd: float, kp_range: float, ki_range: float, kd_range: float, test_pid: Callable[[float, float, float], float]):
        """
        Creates a PIDTuner type instance with initial PID coefficients and ranges, along with a callable supplier for returning pid controller performance error. 
        param: base_kp [float] The specified initial proportional coefficient
        param: base_ki [float] The specified initial integral coefficient 
        param: base_kd [float] The specified initial derivative coefficient
        param: kp_range [float] The specified range from base kp potential testing values are bounded
        param: ki_range [float] The specified range from base ki potential testing values are bounded
        param: kd_range [float] The specified range from base kd potential testing values are bounded
        param: test_pid [Callable[[float, float, float], float]] The specified callable supplier to handle running the pid controller and returning performance error
        """
        # set base values and bounds
        self.init_kp, self.init_ki, self.init_kd = base_kp, base_ki, base_kd # set initial values
        self.init_kp_range, self.init_ki_range, self.init_kd_range = kp_range, ki_range, kd_range
        self.kp_domain = (max(0, base_kp - kp_range), base_kp + kp_range) # set domains as the sets of all possible coefficients
        self.ki_domain = (max(0, base_ki - ki_range), base_ki + ki_range)
        self.kd_domain = (max(0, base_kd - kd_range), base_kd + kd_range)

        self.opt_kp, self.opt_ki, self.opt_kd = base_kp, base_ki, base_kd # optimal coefficients currently set before/during tuning
        self.kp_range, self.ki_range, self.kd_range = kp_range, ki_range, kd_range # testing range from current optimal coefficients
        self.test_pid_def = test_pid # callable test for returning pid error

        # error logging file names:
        self.error_file = ""
        
    def kEvalArgs(self):
        """
        Returns a dictionary mapping for value to test for error evaluations given the base value and range of a coefficient with respect to 
        the which coefficient is being tested (kp, ki, kd) and which test case (base, lower, upper cases). The mapping expects a vector key 
        of the format: (<test-case>, <coefficient-being-tested>, <coefficient-value-to-get>). The case and what being tested matters, then 
        which coefficient to find is specified.

        returns: a dictionary mapping for value to test for error evaluations given the base value and range of a coefficient with respect to 
        the which coefficient is being tested (kp, ki, kd) and which test case (base, lower, upper cases).
        """
        B, L, U = etype.BASE, etype.LOWER, etype.UPPER
        KP, KI, KD = ktype.KP, ktype.KI, ktype.KD
        kp_base, ki_base, kd_base = self.opt_kp, self.opt_ki, self.opt_kd
        R_p, R_i, R_d = self.kp_range, self.ki_range, self.kd_range 

        # eval type, type to eval (ktype), type to get coefficient of (ktype)
        k_eval_args = {(B, KP, KP): kp_base,           (B, KI, KP):  kp_base,          (B, KD, KP):  kp_base, 
                       (L, KP, KP): kp_base-(R_p/2.0), (L, KI, KP):  kp_base,          (L, KD, KP):  kp_base,  
                       (U, KP, KP): kp_base+(R_p/2.0), (U, KI, KP):  kp_base,          (U, KD, KP):  kp_base,
                        
                       (B, KP, KI):  ki_base,          (B, KI, KI): ki_base,           (B, KD, KI):  ki_base, 
                       (L, KP, KI):  ki_base,          (L, KI, KI): ki_base-(R_i/2.0), (L, KD, KI):  ki_base, 
                       (U, KP, KI):  ki_base,          (U, KI, KI): ki_base+(R_i/2.0), (U, KD, KI):  ki_base,
                        
                       (B, KP, KD):  kd_base,          (B, KI, KD):  kd_base,          (B, KD, KD): kd_base, 
                       (L, KP, KD):  kd_base,          (L, KI, KD):  kd_base,          (L, KD, KD): kd_base-(R_d/2.0), 
                       (U, KP, KD):  kd_base,          (U, KI, KD):  kd_base,          (U, KD, KD): kd_base+(R_d/2.0)} 
        return k_eval_args

    def updateBounds(self, k_type: int, k_base: float, k_range: float):
        """
        Updates the specified coefficient value along with its testing range for tuning. 
        param: k_type [int] The specified coefficient type id
        param: k_base [float] The specified coefficient value
        param: k_range [float] The specified testing range
        """
        if k_type == ktype.KP:
            self.opt_kp = min(self.kp_domain[1], max(self.kp_domain[0], k_base))
            self.kp_range = max(0, k_range)
        elif k_type == ktype.KI:
            self.opt_ki = min(self.ki_domain[1], max(self.ki_domain[0], k_base))
            self.ki_range = max(0, k_range)
        elif k_type == ktype.KD:
            self.opt_kd = min(self.kd_domain[1], max(self.kd_domain[0], k_base))
            self.kd_range = max(0, k_range)

    def evaluate(self, kp: float, ki: float, kd: float, epochs: int) -> float: 
        """
        Evaluates an averaged controller performance error for a given set of pid coefficients. 
        param: kp [float] The specified proportional coefficient
        param: ki [float] The specified integral coefficient
        param: kd [float] The specified derivative coefficient
        param: epochs [int] The specified number of trial tests to average error over 
        returns: the average controller performance error
        """
        if kp < 0 or ki < 0 or kd < 0:
            return 1.0 # assume max error for all negative coefficients; these cases should not be considered valid
        # sum of errors from each sim test evaluated for each epoch
        sum = 0.0
        epochs = max(1, epochs)
        for i in range(0, epochs):
            # compute error 
            sum += self.test_pid_def(kp, ki, kd) 
        return sum / epochs

    def step(self, k_type: int, epochs: int, prompt: str) -> list:
        """
        Computes the optimal value for the specified coefficient while descending error over a number of epochs.
        param: k_type [int] The specified coefficient type id 
        param: epochs [int] The specified number of times to repeat test cases for stepping towards the optimal coefficient value
        returns: A vector of performance error found while stepping 
        """
        # short-hand constants
        eval_epochs = PIDTuner.eval_epochs
        BASE, LOWER, UPPER = etype.BASE, etype.LOWER, etype.UPPER
        KP, KI, KD = ktype.KP, ktype.KI, ktype.KD
        # log errors
        error_log = []

        # step through coefficient testing ranges/bounds and binary search to optimal coefficient by number of epochs 
        for epoch in range(0, epochs):
            # eval-case by PID coefficient testing and lookup mapping for current coefficients:
            args = self.kEvalArgs()
            # map current ranges by k types:
            ranges = {ktype.KP: self.kp_range, ktype.KI: self.ki_range, ktype.KD: self.kd_range}

            pid_base_case = "BASE => ( " + format(float(args[(BASE, k_type, KP)]), '.5f') + ", " + format(float(args[(BASE, k_type, KI)]), '.5f') + ", " + format(float(args[(BASE, k_type, KD)]), '.5f') + " )"
            pid_lower_case = "LOWER => ( " + format(float(args[(LOWER, k_type, KP)]), '.5f') + ", " + format(float(args[(LOWER, k_type, KI)]), '.5f') + ", " + format(float(args[(LOWER, k_type, KD)]), '.5f') + " )"
            pid_upper_case = "UPPER => ( " + format(float(args[(UPPER, k_type, KP)]), '.5f') + ", " + format(float(args[(UPPER, k_type, KI)]), '.5f') + ", " + format(float(args[(UPPER, k_type, KD)]), '.5f') + " )"
            rospy.loginfo("PIDTuner.py: Starting next step_epoch. PID (kp,ki,kd) cases: \n" + pid_base_case + ", " + pid_lower_case + ", " + pid_upper_case)

            # compute error for all cases:
            base_error = self.evaluate(kp=args[(BASE, k_type, KP)], ki=args[(BASE, k_type, KI)], kd=args[(BASE, k_type, KD)], epochs=eval_epochs)
            base_output = "PIDTuner.py: Base Err: " + format(100*base_error, '.5f') + "%"
            rospy.loginfo(f"{handler.str_bold_start}{base_output}{handler.str_bold_end}")
            
            lower_error = self.evaluate(kp=args[(LOWER, k_type, KP)], ki=args[(LOWER, k_type, KI)], kd=args[(LOWER, k_type, KD)], epochs=eval_epochs)
            lower_output = "PIDTuner.py: Lower Err: " + format(100*lower_error, '.5f') + "%"
            rospy.loginfo(f"{handler.str_bold_start}{lower_output}{handler.str_bold_end}")

            upper_error = self.evaluate(kp=args[(UPPER, k_type, KP)], ki=args[(UPPER, k_type, KI)], kd=args[(UPPER, k_type, KD)], epochs=eval_epochs)
            upper_output = "PIDTuner.py: Upper Err: " + format(100*upper_error, '.5f') + "%"
            rospy.loginfo(f"{handler.str_bold_start}{upper_output}{handler.str_bold_end}")

            # record minimum
            min_error = min(base_error, min(lower_error, upper_error))
            # log error
            error_log.append(min_error)
            # step in the direction of the desirable bound; else stay at the base case
            if upper_error == min_error: 
                # upper bound is most desirable; step in that direction and re-test
                self.updateBounds(k_type=k_type, k_base=args[(UPPER, k_type, k_type)], k_range=ranges[k_type]/2)
            elif lower_error == min_error:
                # lower bound is most desirable; step in that direction and re-test
                self.updateBounds(k_type=k_type, k_base=args[(LOWER, k_type, k_type)], k_range=ranges[k_type]/2)
            step_prompt = prompt + "step_epoch=" + str(epoch+1) + " of " + str(epochs) + " : completion=" + format(100.0*(float(epoch+1)/float(epochs)), '.5f') + "%"
            pid = "( "+ format(float(self.opt_kp), '.5f') + ", " + format(float(self.opt_ki), '.5f') + ", " + format(float(self.opt_kd), '.5f') + " )"
            rospy.loginfo("PIDTuner.py: PID=" + pid + " => " + step_prompt)
        # return error log
        return error_log
        
    def setErrorLog(self, root_dir: str, error_file: str): 
        """
        Sets the file to act as logger for controller performance error.
        param: root_dir [str] The specified root directory of the file
        param: error_file [str] The specified file for recorded error
        """
        self.error_file = root_dir + error_file

    def logError(self, error_file: str, error_data: list):
        """
        Writes controller performance error logged to specified file destination.
        param: error_file [str] The specified file to write to
        param: error_data [list] The specified list of data 
        """
        with open(error_file, "a") as file:
            for error in error_data:
                file.write(str(error) + "\n")

    def tune(self, kp_epochs: int, ki_epochs: int, kd_epochs: int, tune_epochs: int, log_error: bool):
        """
        Tunes PID controller while stepping through each coefficient for tuning by at most the number of epochs specified for 
        each respective coefficient; writes recorded error to logging file if specified to. 
        param: kp_epochs [int] The specified max number of epochs for stepping through kp tunning a single time
        param: ki_epochs [int] The specified max number of epochs for stepping through ki tunning a single time
        param: kd_epochs [int] The specified max number of epochs for stepping through kd tunning a single time
        param: tune_epochs [int]The specified number of epochs to call this tuning process
        param: log_error [bool] Enables writing performance error of each coefficient to respective logging files
        """

        for epoch in range(0, tune_epochs):
            main_error_log = [] 
            # tuning phase #1 => reset ranges for all coefficients (kp, ki, and kd) for first few phases
            kp_error = self.step(k_type=ktype.KP, epochs=int(max(1, kp_epochs)), prompt="[kp-test, phase 1.1] : ")
            kd_error = self.step(k_type=ktype.KD, epochs=int(max(1, kd_epochs)), prompt="[kd-test, phase 1.2] : ")
            main_error_log = kp_error + kd_error
            self.resetRange(1.0)

            # tuning phase #2
            kp_error = self.step(k_type=ktype.KP, epochs=int(max(1, kp_epochs)), prompt="[kp-test, phase 2.1] : ")
            kd_error = self.step(k_type=ktype.KD, epochs=int(max(1, kd_epochs)), prompt="[kd-test, phase 2.2] : ")
            main_error_log = main_error_log + kp_error + kd_error
            self.resetRange(1.0)

            # tuning phase #3
            kp_error = self.step(k_type=ktype.KP, epochs=int(max(1, kp_epochs)), prompt="[kp-test, phase 3.1] : ")
            kd_error = self.step(k_type=ktype.KD, epochs=int(max(1, kd_epochs)), prompt="[kd-test, phase 3.2] : ")
            ki_error = self.step(k_type=ktype.KI, epochs=int(max(1, ki_epochs)), prompt="[ki-test, phase 3.3] : ")
            main_error_log = main_error_log + kp_error + kd_error + ki_error       
            self.resetRange(1.0)

            # tuning phase #4 
            kp_error = self.step(k_type=ktype.KP, epochs=int(max(1, kp_epochs)), prompt="[kp-test, phase 4.1] : ")
            kd_error = self.step(k_type=ktype.KD, epochs=int(max(1, kd_epochs)), prompt="[kd-test, phase 4.2] : ")
            main_error_log = main_error_log + kp_error + kd_error
            self.resetRange(1.0)

            # record log errors
            if log_error:
                self.logError(self.error_file, main_error_log)
        # return tuned coefficients
        return (self.opt_kp, self.opt_ki, self.opt_kd)

    def resetRange(self, multiplier: float):
        """
        Resets pid coefficient ranges to their initial values before any tuning
        param: multiplier [float] Resets the range to initial values multiplied by the specified value
        """
        self.kp_range = self.init_kp_range * multiplier
        self.ki_range = self.init_ki_range * multiplier
        self.kd_range = self.init_kd_range * multiplier

    def reset(self):
        """
        Resets pid coefficients and ranges to their initial values before any tuning. 
        """
        self.opt_kp = self.init_kp
        self.opt_ki = self.init_ki
        self.opt_kd = self.init_kd    
        self.resetRange(1.0)
