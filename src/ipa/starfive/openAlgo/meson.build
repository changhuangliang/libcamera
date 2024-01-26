# SPDX-License-Identifier: CC0-1.0

algorithms_source = files([
    'open_algo.cpp',
    'starfive_controls.cpp',
])

#open_algorithms_lib = shared_module('starfive_isp_algorithms',
open_algorithms_lib = shared_library('starfive_isp_algorithms',
                           algorithms_source,
                           name_prefix : '',
                           include_directories : [ipa_includes, libipa_includes],
                           dependencies : libcamera_private,
#                           link_with : 'libipa',
                           install : true,
                           install_dir : get_option('libdir'),
                           override_options: ['b_lundef=false'])

