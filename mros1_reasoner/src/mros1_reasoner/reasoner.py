#!/usr/bin/env python
###########################################
# 
# authors:    M.A.GarzonOviedo@tudelft.nl
#             c.h.corbato@tudelft.nl
#             e.aguado@upm.es
##########################################

from collections import defaultdict
import rospy
import argparse
from decimal import Decimal

import signal, sys
from threading import Lock

from reasoner.tomasys import *
from owlready2 import sync_reasoner_pellet, destroy_entity

class Reasoner(object):
    """docstring for Reasoner."""

    def __init__(self):
        super(Reasoner, self).__init__()
        # Initialize global variables
        self.tomasys = None    # owl model with the tomasys ontology
        self.onto = None       # owl model with the application model as individuals of tomasys classes
        self.mock = True       # whether we are running a mock system (True), so reasoning happens in isolation, or connected to the real system
        self.grounded_configuration = None  # name of the current system configuration, as stored in KB
                                            # TODO move to RosReasoner or remove: there can be multiple configurations grounded (for multiple objectives)
        # This Lock is used to ensure safety of tQAvalues
        self.lock = Lock()

        signal.signal(signal.SIGINT, self.save_ontology_exit)


    def load_tomasys_from_file(self, tomasys_file):
        """ Loads tomays ontology from a given file path
        """
        self.tomasys = loadKB_from_file(tomasys_file)

    def load_onto_from_file(self, onto_file):
        """ Loads ontology from a given file path
        """
        self.onto = loadKB_from_file(onto_file)

    def search_objectives(self):
        #Root objectives
        objectives = self.onto.search(type=self.tomasys.Objective)
        return objectives

    def get_new_tomasys_objective(self, objetive_name, iri_seed):
        """Creates Objective individual in the KB given a desired name and a string seed for the Function name
        """
        objective = self.tomasys.Objective(str(objetive_name), namespace=self.onto,
        typeF=self.onto.search_one(iri=str(iri_seed)))
        return objective

    def set_new_grounding(self, fd_name, objective):
        """Given a string fd_name with the name of a FunctionDesign and an objective, removes the previous fg for the objective and ground a new fg of typeF fd
        """
        remove_objective_grounding(objective, self.tomasys, self.onto)
        fd = self.onto.search_one(iri="*{}".format(fd_name), is_a = self.tomasys.FunctionDesign)       
        ground_fd(fd, objective, self.tomasys, self.onto)
        resetObjStatus(objective)
        return str(fd.name)

    # the DiagnosticStatus message process contains, per field
    # - message: "binding_error"
    # - name: name of the fg reported, as named in the OWL file
    # - level: values 0 and 1 are mapped to nothing, values 2 or 3 are mapper to fg.status="INTERNAL_ERROR"
    def updateBinding(self, diagnostic_status):
        fg = onto.search_one(iri="*{}".format(diagnostic_status.name))
        if fg == None:
            return -1
        if diagnostic_status.level > 1:
            fg.fg_status = "INTERNAL_ERROR"
            return 1
        else:
            return 0

    def updateComponentStatus(self, diagnostic_status):
        # Find the Component with the same name that the one in the Component Status message (in diagnostic_status.key)
        component_type = self.onto.search_one(iri="*{}".format(diagnostic_status.name), type=self.tomasys.Component)
        return_value = "NONE"
        if component_type != None:
            value = False
            with self.lock:
                #resetFDRealisability(self.tomasys, self.onto, diagnostic_status.name)
                component_type.c_status = value
        return return_value

    def updateObjectiveStatus(self, diagnostic_status):
        function_type = self.onto.search_one(iri="*{}".format(diagnostic_status.name))
        return_value = "NONE"
        if function_type != None:
            objectives = self.search_objectives()
            old_fg = self.onto.search_one(solvesO=objectives[0])
            o = objectives[0]
            if not (o.typeF == function_type):
            # if movement direction diferent from current direction, change objective
                destroy_entity(o)
                destroy_entity(old_fg)
                # create new objective
                obj_navigate = self.get_new_tomasys_objective("o_"+function_type.name.replace('f_', ''), "*"+function_type.name)
                fd = obtainBestFunctionDesign(obj_navigate,self.tomasys)
                self.grounded_configuration = self.set_new_grounding(fd, obj_navigate)
                rospy.loginfo("\n\nNew objective created to solve function: {0}, FG:{1}".format(function_type.name, self.onto.search(type=self.tomasys.FunctionGrounding)))
                return_value = fd #need reconfig        
        return return_value

    # EXEC REASONING to update ontology with inferences
    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    def perform_reasoning(self):
        return_value = False
        with self.lock:
            with self.onto:
                try:
                    sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)
                    return_value = True
                except Exception as err:
                    raise err
                    self.onto.save(file="error.owl", format="rdfxml")
        return return_value

    # For debugging purposes: saves state of the KB in an ontology file
    # TODO move to library
    # TODO save file in a temp location
    def save_ontology_exit(self, signal, frame):
        self.onto.save(file="error.owl", format="rdfxml")
        sys.exit(0)
