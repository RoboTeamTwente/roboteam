file(REMOVE_RECURSE
  "input_automoc.cpp"
  "sender_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/roboteam_msgs_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
