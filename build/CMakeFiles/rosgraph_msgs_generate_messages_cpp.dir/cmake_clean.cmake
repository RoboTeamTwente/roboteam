file(REMOVE_RECURSE
  "input_automoc.cpp"
  "sender_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rosgraph_msgs_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
