# SPDX-License-Identifier: CC0-1.0

subdir('openAlgo')

ipa_name = 'ipa_starfive'

starfive_ipa_sources = files([
    'starfive.cpp',
    'sensor_helper.cpp',
    'sensor_helper_imx219.cpp',
])

mod = shared_module(ipa_name,
                    [starfive_ipa_sources, libcamera_generated_ipa_headers],
                    name_prefix : '',
                    include_directories : [ipa_includes, libipa_includes],
                    dependencies : libcamera_private,
                    link_with : [libipa, open_algorithms_lib],
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@'],
                  install : false,
                  build_by_default : true)
endif

subdir('data')
#subdir('algorithms')
