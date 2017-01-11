#!/bin/bash

#CONFIG


openhabTargetDIR=$(pwd)

openhabURL="https://bintray.com/artifact/download/openhab/bin/distribution-1.8.3-runtime.zip"

habminBindingURL="https://github.com/cdjackson/HABmin/raw/master/addons/org.openhab.io.habmin-1.7.0-SNAPSHOT.jar"
habminBindingTargetDIR="${openhabTargetDIR}/addons"

openhabAppURL="https://github.com/cdjackson/HABmin/archive/master.zip"
habminAppTargetDIR="${openhabTargetDIR}/webapps/habmin"

weblogsTargetDIR="${openhabTargetDIR}/webapps/weblogs"

addOnsURL="https://bintray.com/artifact/download/openhab/bin/distribution-1.8.3-addons.zip"
addOnsTargetDIR="${openhabTargetDIR}/all-addons"

zwaveBindingURL="https://openhab.ci.cloudbees.com/job/openHAB1-Addons/lastSuccessfulBuild/artifact/bundles/binding/org.openhab.binding.zwave/target/org.openhab.binding.zwave-1.9.0-SNAPSHOT.jar"
zwqveBindingTargetDIR="${openhabTargetDIR}/addons"


httpBinding="org.openhab.binding.http-1.8.3.jar"
execBinding="org.openhab.binding.exec-1.8.3.jar"
mongodbBinding="org.openhab.persistence.mongodb-1.8.3.jar"

echo "============================================"
echo "Openhab 1.8.3 install script"
echo ""
echo "Remember to have java and mongodb installed "
echo "============================================"

echo "============================================"
echo "Downloading openhab"
echo "============================================"
wget "${openhabURL}" -O /tmp/openhab-1.8.3.zip

echo "============================================"
echo "Extracting to openhab folder ${openhabTargetDIR}"
echo "============================================"
unzip  -n  /tmp/openhab-1.8.3.zip -d "${openhabTargetDIR}"
rm /tmp/openhab-1.8.3.zip

echo "============================================"
echo " Downloading habmin binding to openhab/addons"
echo "============================================"
wget "${habminBindingURL}" -P "${habminBindingTargetDIR}"

echo "============================================"
echo " Downloading habmin web interface"
echo "============================================"
wget "${openhabAppURL}" -O /tmp/habmin-1.7.0.zip

echo "============================================"
echo " Extracting habmin app to openhab/webapps/habmin"
echo "============================================"
unzip  -n  /tmp/habmin-1.7.0.zip  -d /tmp 
mv  /tmp/HABmin-master $habminAppTargetDIR
rm /tmp/habmin-1.7.0.zip 
##########################
echo "============================================"
echo " Downloading zwave binding to openhab/addons"
echo "============================================"
wget "${zwaveBindingURL}" -P "${habminBindingTargetDIR}"

############################


echo "============================================"
echo " Downloading rest of addons"
echo "============================================"
wget "${addOnsURL}" -O /tmp/1.8.3-addons.zip

echo "============================================"
echo " Extracting them to openhab/all-addons"
echo "============================================"
mkdir -p "${addOnsTargetDIR}"
unzip  -n /tmp/1.8.3-addons.zip -d "${addOnsTargetDIR}"
rm /tmp/1.8.3-addons.zip

# for logs on web interface...
mkdir -p "${weblogsTargetDIR}"


echo "============================================"
echo "Copying used addons from openhab/all-addons to openhab/addons"
echo "============================================"
cp "${addOnsTargetDIR}/${httpBinding}" "${habminBindingTargetDIR}"
cp "${addOnsTargetDIR}/${execBinding}" "${habminBindingTargetDIR}"  
cp "${addOnsTargetDIR}/${mongodbBinding}" "${habminBindingTargetDIR}"

echo "============================================"
echo "Setting up an initial item configuration "
echo "============================================"
cp "${openhabTargetDIR}/configurations/items/test.items_UOL" "${openhabTargetDIR}/configurations/items/test.items"


echo "============================================"
echo "Configuring openhab daemon"
echo "============================================"

cp ${openhabTargetDIR}/openhab ${openhabTargetDIR}/openhab_tmp

ORIG="RUN_AS=ambient"
NEW="RUN_AS="$(whoami)
sed -i -e "s/$ORIG/$NEW/g" ${openhabTargetDIR}/openhab_tmp

ORIG="ECLIPSEHOME=\"/home/ambient/catkin_ws/src/ENRICHME/codes/ais/openhab\";"
NEW="ECLIPSEHOME=\""${openhabTargetDIR}"\";"
sed -i -e 's|'$ORIG'|'$NEW'|g' ${openhabTargetDIR}/openhab_tmp

echo "===================================================="
echo " Adding openhab to boot up services (runlevels 2345)"
echo "===================================================="
sudo mv "${openhabTargetDIR}/openhab_tmp" /etc/init.d/openhab
sudo chmod a+x /etc/init.d/openhab
sudo update-rc.d openhab defaults

echo "============================================"
echo " To launch openhab now, run:"
echo "    sudo service openhab start"
echo "============================================"


