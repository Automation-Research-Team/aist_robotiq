/*!
 *  \file	Localizer.h
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <PhoLocalization.h>
#include <iostream>
#include <memory>

#include <PhoXi.h>

namespace aist_photoneo_localization
{
/************************************************************************
*  class Localizar							*
************************************************************************/
class Localizer
{
  private:
    using localization_t = pho::sdk::PhoLocalization;
    using scene_source_t = pho::sdk::ScaneSource;

  public:
    Localizer()	:_localization(new localization_t())	{}

    void	set_scene(const pho::api::PPhoXi& camera)
		{
		    _scene = scene_source_t::PhoXi(camera);
		    _localization->SetSceneSource(_scene);
		}

    void	set_scene(const std::string& ply_file)
		{
		    _scene = scene_source_t::File(ply_file);
		    _localization->SetSceneSource(_scene);
		}

    void	set_config(const std::string& plcf_file)
		{
		    _localization->LoadLocalizationConfiguration(plcf_file);
		}


  private:
    std::unique_ptr<localization_t>	_localization;
    scene_source_t			_scene;
};

}	// namespace aist_photoneo_localization
#endif	// !LOCALIZER_H
