import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String

from reasoner.reasoner import Reasoner
from reasoner.tomasys import obtainBestFunctionDesign, print_ontology_status
from reasoner.tomasys import evaluateObjectives, loadKB_from_file, destroy_entity
from reasoner.tomasys import remove_objective_grounding

from reasoner.msg import matrixAllocation
import csv
import numpy as np
class RosReasoner(Reasoner):
    """docstring for RosComponents."""
    def __init__(self):

        super(RosReasoner, self).__init__()
        self.initialized = False
        # Start rosnode
        rospy.init_node('reasoner_node', anonymous=True)
        self.reconfig_pub = rospy.Publisher('/reconfiguration', matrixAllocation, queue_size=1)
        # Get ontology and tomasys file paths from parameters
        model_file = self.check_and_read_parameter('~model_file')
        tomasys_file =  self.check_and_read_parameter('~tomasys_file')
        self.reconfiguration_path = self.check_and_read_parameter('~reconfiguration_path')
        self.grounded_configuration = self.check_and_read_parameter('~desired_configuration')
        #Start interfaces
        sub_diagnostics = rospy.Subscriber("/diagnostics", DiagnosticArray, self.callbackDiagnostics)

        # load tomasys
        if tomasys_file is not None:
            self.load_tomasys_from_file(tomasys_file)
            if self.tomasys is not None:
                rospy.loginfo("Loaded tomasys: %s", str(tomasys_file))
            else:
                rospy.logerr("Failed to load tomasys from: %s", str(tomasys_file))
                return
        else:
                return

        #rospy.sleep(0.5) # Wait for subscribers (only for the test_1_level_functional_architecture)

        # load ontology
        if model_file is not None:
            self.load_onto_from_file(model_file)
            if self.onto is not None:
                rospy.loginfo("Loaded ontology: %s", str(model_file))
            else:
                rospy.logerr("Failed to load ontology from: %s", str(model_file))
                return
        else:
                return

        if self.grounded_configuration is not None:
            rospy.loginfo('grounded_configuration initialized to: %s', self.grounded_configuration)
        else:
            rospy.logwarn('grounded_configuration not found in the param server')

        self.initialized = True
        # Reasoner initialization completed
        rospy.loginfo("[RosReasoner] -- Reasoner Initialization Ok")

    def start_timer(self):
        timer_rate =  self.check_and_read_parameter('~reasoning_rate', 2.0)
        timer = rospy.Timer(rospy.Duration(timer_rate), self.timer_cb)
       
    def check_and_read_parameter(self, param_name, default_value=None):
        """ Checks if a parameter exists and returns its value
            Args:
                    param_name (string): The name of the parameter.
            Returns:
                    The parameter value if it exists, None otherwise.
        """
        if rospy.has_param(str(param_name)):
            return rospy.get_param(str(param_name))
        else:
            rospy.logwarn("Parameter \'%s\' not defined! - Returning %s", str(param_name), str(default_value))
            return default_value

# NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
    # - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam
    def initKB(self):

        rospy.loginfo('KB initialization')

        objectives = self.search_objectives()

        # if no objectives in the OWL file, standard navigation objective is assumed
        if objectives == []:
            rospy.loginfo('Creating default Objective')
            #obj_navigate = self.get_new_tomasys_objective("o_nav_surge", "*f_nav_surge")
            obj_navigate = self.get_new_tomasys_objective("o_nav_heave", "*f_nav_heave")
            # # Function Groundings and Objectives
            self.set_new_grounding(self.grounded_configuration, obj_navigate)
            self.request_configuration("fd_heave_all")
        elif len(objectives) == 1:
            o = objectives[0]
            fd = obtainBestFunctionDesign(o, self.tomasys)
            self.set_new_grounding(fd.name, o)
            rospy.logwarn('Objective and initial FG are generated from the OWL file')
        else:
            rospy.logerr('Metacontrol cannot handle more than one Objective in the OWL file (the Root Objective)')

        # For debugging InConsistent ontology errors, save the ontology before reasoning
        #self.onto.save(file="tmp_debug.owl", format="rdfxml")

    def callbackDiagnostics(self, msg):
        if self.onto is not None:
            for diagnostic_status in msg.status:
                if diagnostic_status.message == "Component status":
                    rospy.logwarn("New component status received")
                    up_cs = self.updateComponentStatus(diagnostic_status)
                    if up_cs is not "NONE":
                        rospy.loginfo("\n\nCS Message received!\t Component affected: {0}".format(diagnostic_status.name))
                        #request_configuration(up_cs)
                elif diagnostic_status.message == "Movement status":
                    rospy.logwarn("New movement status received")
                    up_mov = self.updateObjectiveStatus(diagnostic_status)
                    if up_mov is not "NONE":
                        rospy.loginfo("\n\nMovement message received!\t Movement affected: {0}".format(diagnostic_status.name))
                        self.request_configuration(up_mov)
                   

    def read_config_file (self, fd):
        reader = np.genfromtxt(self.reconfiguration_path+fd + ".csv",  delimiter = ",")
        return reader

    def request_configuration(self, fd):
        rospy.logwarn_throttle(1., 'New Configuration requested: {}'.format(fd))
        ###PUBLISH B matrix
        matrix = self.read_config_file(fd)
        msg = matrixAllocation()
        msg.X = matrix[0].astype(np.double)
        msg.Y = matrix[1].astype(np.double)
        msg.Z = matrix[2].astype(np.double)
        msg.K = matrix[3].astype(np.double)
        msg.M = matrix[4].astype(np.double)
        msg.N = matrix[5].astype(np.double)
        self.reconfig_pub.publish(msg)
        rospy.logwarn("= RECONFIGURATION SUCCEEDED =")
        #result = 1
        #return result

    ## main metacontrol loop
    def timer_cb(self, event):

        rospy.loginfo('Entered timer_cb for metacontrol reasoning')
        rospy.loginfo('  >> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        if self.perform_reasoning():
            rospy.loginfo('     >> Finished ontological reasoning)')
        else:
            rospy.logerr("Reasoning error")

        # PRINT system status
        print_ontology_status(self.tomasys)

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_internal_error = evaluateObjectives(self.tomasys)
        if not objectives_internal_error:
            rospy.loginfo("No Objectives in status ERROR: no adaptation is needed")
            #rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')
            #rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        elif len(objectives_internal_error) > 1 :
            rospy.logerr("- More than 1 objectives in error, case not supported yet.")
            rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')
            rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        else:
            rospy.logwarn("Objectives in status ERROR: {}".format([o.name for o in objectives_internal_error]) )
            rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')

        # ADAPT MAPE -Plan & Execute
        rospy.loginfo('  >> Started MAPE-K ** PLAN adaptation **')
        o = objectives_internal_error[0]
        rospy.loginfo("=> Reasoner searches FD for objective: {}".format(o.name) )
        fd = obtainBestFunctionDesign(o, self.tomasys)
        if not fd:
            rospy.logerr(
                "No FD found to solve Objective {}".format(o.name)) # for DEBUGGING in csv
            rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        rospy.loginfo('  >> Finished MAPE-K ** Plan adaptation **')
        # request new configuration
        rospy.loginfo('  >> Started MAPE-K ** EXECUTION **')
        self.grounded_configuration = self.set_new_grounding(fd, o) # Set new grounded_configuration
        self.request_configuration(fd)
        rospy.loginfo('  >> Finished MAPE-K ** EXECUTION **')
        # Process adaptation feedback to update KB:
        rospy.loginfo('Exited timer_cb for metacontrol reasoning')
