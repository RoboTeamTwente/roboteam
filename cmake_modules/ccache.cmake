# Find programs and assign the boolean to the variables, find_program(variable binary)
find_program(CCACHE ccache)

if(CCACHE)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) #probably less useful for linking
    message("Found ccache")
else()
    message("Could not find CCACHE! We highly recommend installing it for quicker builds")
endif(CCACHE)

