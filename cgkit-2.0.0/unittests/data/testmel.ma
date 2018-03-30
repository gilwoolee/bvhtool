requires maya "7.0";
currentUnit -l centimeter -a degree -t film;
fileInfo "application" "maya";
fileInfo "product" "Maya Complete 7.0";
fileInfo "version" "7.0";
fileInfo "cutIdentifier" "200507192211-654274";
fileInfo "osv" "Microsoft Windows XP Service Pack 2 (Build 2600)\n";
createNode transform -n "pCube1";
	setAttr ".t" -type "double3" -2.1707668811066378 0 0 ;
createNode mesh -n "pCubeShape1" -p "pCube1";
	setAttr -k off ".v" -cb true -ca false;
	setAttr ".vir" yes;
	setAttr ".vif" yes;
	setAttr ".uvst[0].uvsn" -type "string" "map1";
	setAttr ".cuvs" -type "string" "map1";
	setAttr ".dcc" -type "string" "Ambient+Diffuse";
	setAttr ".covm[0]"  0 1 1;
	setAttr ".cdvm[0]"  0 1 1;
select -ne :time1;
connectAttr "polyCube1.out" "pCubeShape1.i";
disconnectAttr "polyCube1.out" "pCubeShape1.i";
