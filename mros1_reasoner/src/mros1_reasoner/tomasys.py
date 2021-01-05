###########################################
#
# authors:  c.h.corbato@tudelft.nl
#           M.A.GarzonOviedo@tudelft.nl
#           e.aguado@upm.es
#
# DESCRIPTION:
#  Python library implementing utilities to manipulate Knowledge Bases
#  based on the tomasys metamodel (Tbox), using the owlready2 library
##########################################

from owlready2 import get_ontology, destroy_entity
import logging
import rospy


# Returns
# - kb_box: the ontology readed
def loadKB_from_file(kb_file):
    """ Reads a KB from a given file
        (Replaces loadTomasysKB)
        Args:
                kb_file (string): Full path to the ontology to be loaded.
        Returns:
                kb_box (ontology): Ontology readed, None: if there is an error.
    """

    try:
        kb_box = get_ontology(kb_file).load()
    except Exception as e:
        logging.exception("{0}".format(e))
        return None
    return kb_box
# # Returns
# # - tbox: the ontology containing the Tbox frmo the tomasys.owl
# # - abox: the ontology containing the individuals to initialize the KB, aka the abox
# def loadTomasysKB(tboxfile, abox_file):
#     onto_path.append(os.path.dirname(os.path.realpath(tboxfile)))
#     onto_path.append(os.path.dirname(os.path.realpath(abox_file)))
#     tbox = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file (that does not seem to work)
#     abox = get_ontology(abox_file).load()
#     return tbox, abox

# To reset the individuals that no longer hold due to adaptation
# for the moment, only Objective individuals statuses
# - tomasys: ontology holding the Tbox

# ESTHER ---- no uso resetObjStatus, resetFDRealisability
def resetObjStatus(objective, status=None): 
         # logging.warning("\nReseting obj {0}".format(objective.name))
    objective.o_status = status


# def resetFDRealisability(tbox, abox, c_name):
#     logging.warning("\nReset realisability:\n")
#     component = abox.search_one(iri="*{}".format(c_name))
#     if component is None:
#         # logging.warning("C not found Return\n\n\n")
#         return
        
#     if component.c_status is None:
#         # logging.warning("C status None Return\n\n\n")
#         return
#     else:
#         if component.c_status in ["FALSE"]:
#             logging.warning("component statis is {} - Set to None\n".format(component.c_status))
#             for fd in list(tbox.FunctionDesign.instances()):
#                 if fd.fd_realisability is None:
#                     continue
#                 else:
#                     logging.warning("FD {0} realisability: {1} -  Set to None".format(fd.name, fd.fd_realisability))
#                     fd.fd_realisability = None
    
# ---- FIN ESTHER
# For debugging purposes
def print_ontology_status(kb_box):

    logging.warning("\t\t\t >>> Ontology Status   <<<")

    logging.warning("\n\tComponent Status:\t{0}".format([(c.name, c.c_status) for c in list(kb_box.ComponentState.instances())]))
    
    # logging.warning("\nFD Realizability Statuses:\n")
    # for fd in list(kb_box.FunctionDesign.instances()):
    #     logging.warning("FD {0} realisability: {1}".format(fd.name, fd.fd_realisability))

    for i in list(kb_box.FunctionGrounding.instances()):
        logging.warning("\n\tFG: {0}  Status: {1}  Solves: {2}  FD: {3}".format(i.name, i.fg_status, i.solvesO.name,  i.typeFD.name))

    for i in list(kb_box.Objective.instances()):
        logging.warning("\n\tOBJECTIVE: {0}   Status: {1}".format(i.name, i.o_status))
    logging.warning("\t\t\t >>>>>>>>>>>>> <<<<<<<<<<<")


# Evaluates the Objective individuals in the KB and returns a list with those in error
def evaluateObjectives(tbox):
    objectives_internal_error = []
    for o in list(tbox.Objective.instances() ):
        if o.o_status == "INTERNAL_ERROR":
            objectives_internal_error.append(o)
    return objectives_internal_error

# Select best FD in the KB, given:
# - o: individual of tomasys:Objective
# - tomasys ontology that contains the tomasys tbox
def obtainBestFunctionDesign(o, tbox):
    logging.warning("\t\t\t == Obatin Best Function Design ==")
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tbox.FunctionDesign.instances()):
        if fd.solvesF == f:
            fds.append(fd)
    # fiter fds to only those available
    # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
    realisable_fds = [fd for fd in fds if fd.fd_realisability != False]
    logging.warning("== FunctionDesigns REALISABLE: %s", str([fd.name for fd in realisable_fds]))
    # discard FDs already grounded for this objective when objective in error
    suitable_fds= [fd for fd in fds if ((not o in fd.fd_error_log) and (fd.fd_realisability != False))]
    logging.warning("== FunctionDesigns AVAILABLE: %s", str([fd.name for fd in suitable_fds]))
    # get best FD based on higher Utility/trade-off of QAs
    if suitable_fds != []:
        best_utility = 0
        # best_fd = fds_for_obj[0]
        for fd in suitable_fds:
            utility_fd = utility(fd)
            logging.warning("== Utility for %s : %f", fd.name, utility_fd)
            if  utility_fd > best_utility:
                best_fd = fd
                best_utility = utility_fd

        logging.warning("\t\t\t == Best FD available %s", str(best_fd.name))
        return best_fd.name
    else:
        logging.warning("\t\t\t == *** NO SOLUTION FOUND ***")
        return None



def ground_fd(fd, objective, tbox, abox):
    """Given a FunctionDesign fd and an Objective objective, creates an individual FunctionGrounds with typeF fd and solve) objective
        returns the fg
    """
    fg = tbox.FunctionGrounding("fg_"+fd.name.replace('fd_', ''), namespace=abox, typeFD=fd, solvesO=objective)
    # TODO: ground objectives required by FD
    return fg

def remove_objective_grounding(objective, tbox, abox):
    """Given an objective individual, removes the grounded hierarchy (fg tree) that solves it.
    """
    fg = abox.search_one(solvesO=objective)
    if fg:
        destroy_entity(fg)

def utility(fd):
    # utility is equal to the expected performance
    utility = [qa for qa in fd.hasQAestimation if qa.isQAtype.name == "performance"]
    if len(utility) != 1:
        logging.warning("FD has no expected perfomance or multiple definitions (inconsistent)")
        return 0.001
    else:
        return utility[0].hasValue
