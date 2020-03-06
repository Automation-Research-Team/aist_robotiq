#!/bin/bash

install_pkg()
{
    cd /tmp
    wget -O $2 http://mirrors.kernel.org/ubuntu/pool/main/$1/$2
    dpkg -i $2
    rm $2
}

download_photoneo()
{
    cd /tmp
    wget https://photoneo.com/files/installer/$1/$2
    tar xvf $2
}

install_phoxi_control()
{
    apt-get install -y libpcre16-3
    install_pkg i/icu libicu55_55.1-7_amd64.deb
    install_pkg libp/libpng libpng12-0_1.2.54-1ubuntu1_amd64.deb
    download_photoneo PhoXi/1.2.14 PhotoneoPhoXiControlInstaller-1.2.14-Ubuntu16-STABLE.tar
    /tmp/PhotoneoPhoXiControlInstaller-1.2.14-Ubuntu16-STABLE.run
    rm /tmp/PhotoneoPhoXiControlInstaller-1.2.14-Ubuntu16-STABLE.run
}

install_photoneo_localization()
{
    install_pkg libr/libraw libraw15_0.17.1-1_amd64.deb
    install_pkg j/jasper libjasper1_1.900.1-14ubuntu3.5_amd64.deb
    install_pkg libw/libwebp libwebp5_0.4.4-1_amd64.deb
    install_pkg libw/libwebp libwebpmux1_0.4.4-1_amd64.deb
    download_photoneo Localization/1.2.2 PhotoneoLocalizationSDKInstaller-1.2.2+49076c4-Linux-gcc5.4.0.run.tar
    /tmp/PhotoneoLocalizationSDKInstaller-1.2.2+49076c4-Linux-gcc5.4.0.run
    rm /tmp/PhotoneoLocalizationSDKInstaller-1.2.2+49076c4-Linux-gcc5.4.0.run
}

install_phoxi_control
install_photoneo_localization
