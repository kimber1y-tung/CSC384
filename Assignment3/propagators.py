#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.

'''This file will contain different constraint propagators to be used within
   bt_search.

   propagator == a function with the following template
      propagator(csp, newly_instantiated_variable=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newly_instaniated_variable is an optional argument.
      if newly_instantiated_variable is not None:
          then newly_instantiated_variable is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method).
      bt_search NEEDS to know this in order to correctly restore these
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newly_instantiated_variable = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated
        constraints)
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope
        contains only one variable) and we forward_check these constraints.

        for gac we establish initial GAC by initializing the GAC queue
        with all constaints of the csp


      PROPAGATOR called with newly_instantiated_variable = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

         for gac we initialize the GAC queue with all constraints containing V.


var_ordering == a function with the following template
    var_ordering(csp)
        ==> returns Variable

    csp is a CSP object---the heuristic can use this to get access to the
    variables and constraints of the problem. The assigned variables can be
    accessed via methods, the values assigned can also be accessed.

    var_ordering returns the next Variable to be assigned, as per the definition
    of the heuristic it implements.
   '''
import itertools

from cspbase import CSP, Constraint, Variable


def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no
    propagation at all. Just check fully instantiated constraints'''

    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0: # return the number of unassigned variables in the constraint's scope
            vals = []
            vars = c.get_scope() # get list of variables the constraint is over
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def FCCheck(C, X, pruned):
    """
    C is a constraint with all its variables already
    assigned, except for variable X
    for d := each member of CurDom(X):
        if making X = d together with previous assignments to variables in the scope of C falsifies C:
            remove d from CurDom(X)
    if CurDom[X] == {}:
        RETURN DWO   # Domain Wipe Out
    RETURN ok
    Note: code idea from lecture slides
    """
    # keep track of all pruned variable,value pairs
    for d in X.cur_domain():
        X.assign(d)
        vals = []
        vars = C.get_scope()
        for var in vars:
            vals.append(var.get_assigned_value())
        if not C.check(vals):
            if X.in_cur_domain(d):
                X.prune_value(d)
                pruned.append((X, d))
        X.unassign()

    # dwo
    if X.cur_domain_size() == 0:
        return False, pruned
    return True, pruned

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with
       only one uninstantiated variable. Remember to keep
       track of all pruned variable,value pairs and return '''
    #IMPLEMENT
    # newVar==None,  check all constraints
    if newVar == None:
        FC_constraints = csp.get_all_cons()
    # newVar==var, check constraints containing newVar
    else:
        FC_constraints = csp.get_cons_with_var(newVar)

    pruned = [] # keep track of all pruned variable,value pairs
    for constraint in FC_constraints:
        if constraint.get_n_unasgn() == 1:
            for var in constraint.get_unasgn_vars():

                #FCCheck
                has_solution, pruned = FCCheck(constraint, var, pruned)
                if not has_solution:
                    return False, pruned

    return True, pruned
    # for c in csp.get_cons_with_var(newVar):
    #     if c.get_n_unasgn() == 0:
    #         vals = []
    #         vars = c.get_scope()
    #         for var in vars:
    #             vals.append(var.get_assigned_value())
    #         if not c.check(vals):
    #             return False, []
    # return True, []

def GAC_Enforce(csp, GACQueue, pruned):
    """
    GAC-Queue contains all constraints one of whose variables has
    had its domain reduced. At the root of the search tree we can
    first run GAC_Enforce with all constraints on GAC-Queue
    while GACQueue not empty

    C = GACQueue.extract()
    for V := each member of scope(C)
        for d := CurDom[V]
            Find an assignment A for all other variables in scope(C)
            such that C(A ∪ V=d) is True
            if A not found
                CurDom[V] = CurDom[V] - d   # remove d from the domain of V
                if CurDom[V] == {}    # DWO for V
                    empty GACQueue
                    return DWO        # return immediately
                else
                    push all constraints C’ such that V ∈ scope(C’)
                    and C’ ̸∈ GACQueue on to GACQueue
    return TRUE   # loop exited without DWO
    Note: code idea from lecture slides
    """
    while len(GACQueue) != 0:
        C = GACQueue.pop(0)
        vars = C.get_scope()
        for V in vars:
            for d in V.cur_domain():
                if not C.has_support(V, d):
                    V.prune_value(d)
                    pruned.append((V, d))

                    # dwo
                    if V.cur_domain_size() == 0:
                        return False, pruned

                    else:
                        temp_c = csp.get_cons_with_var(V)
                        for c in temp_c:
                            if c not in GACQueue:
                                GACQueue.append(c)

    return True, pruned

def prop_GAC(csp, newVar=None):
    '''Do GAC propagation. If newVar is None we do initial GAC enforce
       processing all constraints. Otherwise we do GAC enforce with
       constraints containing newVar on GAC Queue'''
    #IMPLEMENT
    # newVar == None, run gac on all constraints
    if newVar == None:
        GAC_constraints = csp.get_all_cons()
    # newVar == var, check constraints containing newVar
    else:
        GAC_constraints = csp.get_cons_with_var(newVar)
    pruned = []
    return GAC_Enforce(csp, GAC_constraints, pruned)

def ord_mrv(csp):
    ''' return variable according to the Minimum Remaining Values heuristic
    branch on smallest remaining values(smallest curdom)'''
    #IMPLEMENT
    min_dom_size = float("inf")
    min_dom_size_var = None
    variables = csp.get_all_vars()
    # variables = csp.get_all_unasgn_vars()
    for variable in variables:
        variable_dom_size = variable.cur_domain_size()
        if variable_dom_size < min_dom_size:
            min_dom_size = variable_dom_size
            min_dom_size_var = variable
    return min_dom_size_var










