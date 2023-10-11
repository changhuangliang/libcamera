# SPDX-License-Identifier: CC0-1.0

sensor_helper_source = files([
    'sensor_helper.cpp',
    'sensor_helper_imx219.cpp',
])

sensor_helper_lib = shared_module('starfive_sensor_helper',
                           sensor_helper_source,
                           name_prefix : '',
                           include_directories : [ipa_includes, libipa_includes],
                           dependencies : libcamera_private,
                           link_with : libipa,
                           install : true,
                           install_dir : get_option('libdir'))

