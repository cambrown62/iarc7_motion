// Thrust model for unknown_10_45_dynamic_r2
// Generated on 2018-03-20 08:37:00.110739

#ifndef PROP_unknown_10_45_dynamic_r2_H
#define PROP_unknown_10_45_dynamic_r2_H

const float response_lag = 0.0299716507441;
const float thrust_min = 0.0;
const float thrust_max = 0.8;
const float voltage_min = 0.0;
const float voltage_max = 11.5;
const float timestep = 0.02;

const uint8_t thrust_to_voltage_order = 3;
const float thrust_to_voltage[4] = {16.349891243165725, -29.722915017680208, 25.612765123529435, 0.0593597204659899};

const uint8_t num_thrust_points = 20;
const uint8_t num_voltage_points = 20;
const float voltage_to_jerk_mapping[20][21][2] = {
    {{0.0, 0}, {0.0, 0.00137203460893}, {0.605263157895, 0.00176925063456}, {1.21052631579, 0.00307579938999}, {1.81578947368, 0.00557296327096}, {2.42105263158, 0.00963413173992}, {3.02631578947, 0.0157533813695}, {3.63157894737, 0.0245780532342}, {4.23684210526, 0.0369384129414}, {4.84210526316, 0.0538558464113}, {5.44736842105, 0.0764884476716}, {6.05263157895, 0.105937336382}, {6.65789473684, 0.142804618852}, {7.26315789474, 0.18643193402}, {7.86842105263, 0.233989510463}, {8.47368421053, 0.280066004541}, {9.07894736842, 0.317666436554}, {9.68421052632, 0.340730101789}, {10.2894736842, 0.346712585664}, {10.8947368421, 0.337313707211}, {11.5, 0.31698424414}},
    {{0.0421052631579, 0}, {0.0, 0.0407124395851}, {0.605263157895, 0.0407517001536}, {1.21052631579, 0.0414902294059}, {1.81578947368, 0.0433960864443}, {2.42105263158, 0.0469215829724}, {3.02631578947, 0.0523811758737}, {3.63157894737, 0.0603611230581}, {4.23684210526, 0.0716036977007}, {4.84210526316, 0.0870106369845}, {5.44736842105, 0.107594901055}, {6.05263157895, 0.134323047068}, {6.65789473684, 0.167769044002}, {7.26315789474, 0.207527526397}, {7.86842105263, 0.251494985032}, {8.47368421053, 0.295452373016}, {9.07894736842, 0.333610205302}, {9.68421052632, 0.360363725929}, {10.2894736842, 0.372441451007}, {10.8947368421, 0.370009942511}, {11.5, 0.356042630126}},
    {{0.0842105263158, 0}, {0.0, 0.0800184259082}, {0.605263157895, 0.079770954605}, {1.21052631579, 0.0801846111875}, {1.81578947368, 0.0814748352263}, {2.42105263158, 0.0840099107445}, {3.02631578947, 0.0889068716786}, {3.63157894737, 0.0962379037187}, {4.23684210526, 0.106641275281}, {4.84210526316, 0.120919192783}, {5.44736842105, 0.139956928616}, {6.05263157895, 0.164587815043}, {6.65789473684, 0.195314395051}, {7.26315789474, 0.231853456515}, {7.86842105263, 0.272583977144}, {8.47368421053, 0.314191812916}, {9.07894736842, 0.351967997793}, {9.68421052632, 0.380999064553}, {10.2894736842, 0.397816794928}, {10.8947368421, 0.401539167645}, {11.5, 0.393809481769}},
    {{0.126315789474, 0}, {0.0, 0.118990862543}, {0.605263157895, 0.118440908914}, {1.21052631579, 0.118516440739}, {1.81578947368, 0.119402945713}, {2.42105263158, 0.121412689341}, {3.02631578947, 0.125112065644}, {3.63157894737, 0.131813458674}, {4.23684210526, 0.141632094246}, {4.84210526316, 0.155119721069}, {5.44736842105, 0.173041455711}, {6.05263157895, 0.196093120526}, {6.65789473684, 0.224672759699}, {7.26315789474, 0.258530313485}, {7.86842105263, 0.296360129383}, {8.47368421053, 0.335536594847}, {9.07894736842, 0.372290467541}, {9.68421052632, 0.402499273997}, {10.2894736842, 0.422869117431}, {10.8947368421, 0.431913909651}, {11.5, 0.430194181643}},
    {{0.168421052632, 0}, {0.0, 0.157599926726}, {0.605263157895, 0.156732794783}, {1.21052631579, 0.156458938815}, {1.81578947368, 0.156936103221}, {2.42105263158, 0.158425770954}, {3.02631578947, 0.161391958328}, {3.63157894737, 0.166739221726}, {4.23684210526, 0.176084737242}, {4.84210526316, 0.189133791888}, {5.44736842105, 0.206362121764}, {6.05263157895, 0.228315019318}, {6.65789473684, 0.255255239257}, {7.26315789474, 0.286901855177}, {7.86842105263, 0.322150092774}, {8.47368421053, 0.358905043044}, {9.07894736842, 0.394205981629}, {9.68421052632, 0.424740316569}, {10.2894736842, 0.447630585268}, {10.8947368421, 0.461164476577}, {11.5, 0.465123534578}},
    {{0.210526315789, 0}, {0.0, 0.195816358484}, {0.605263157895, 0.194618754892}, {1.21052631579, 0.193986626952}, {1.81578947368, 0.19405255572}, {2.42105263158, 0.195032926165}, {3.02631578947, 0.197303376305}, {3.63157894737, 0.201573511267}, {4.23684210526, 0.209354898541}, {4.84210526316, 0.222350457341}, {5.44736842105, 0.23938126307}, {6.05263157895, 0.260753016389}, {6.65789473684, 0.286559380692}, {7.26315789474, 0.316441515521}, {7.86842105263, 0.349421678932}, {8.47368421053, 0.383828245199}, {9.07894736842, 0.417400006846}, {9.68421052632, 0.447608520506}, {10.2894736842, 0.472134841199}, {10.8947368421, 0.489339603385}, {11.5, 0.498547229557}},
    {{0.252631578947, 0}, {0.0, 0.233612850805}, {0.605263157895, 0.232073180042}, {1.21052631579, 0.231076567252}, {1.81578947368, 0.230733333921}, {2.42105263158, 0.231220762144}, {3.02631578947, 0.232839918559}, {3.63157894737, 0.236142101759}, {4.23684210526, 0.242249260231}, {4.84210526316, 0.253815287584}, {5.44736842105, 0.27136668281}, {6.05263157895, 0.292829449904}, {6.65789473684, 0.318090084972}, {7.26315789474, 0.346683654217}, {7.86842105263, 0.37772635158}, {8.47368421053, 0.409911723996}, {9.07894736842, 0.441599201882}, {9.68421052632, 0.47099846359}, {10.2894736842, 0.49641679655}, {10.8947368421, 0.516506484137}, {11.5, 0.530443024778}},
    {{0.294736842105, 0}, {0.0, 0.270965893546}, {0.605263157895, 0.269074436679}, {1.21052631579, 0.267709914388}, {1.81578947368, 0.26696355109}, {2.42105263158, 0.266979672706}, {3.02631578947, 0.267998055666}, {3.63157894737, 0.270444472669}, {4.23684210526, 0.275147970028}, {4.84210526316, 0.283951927951}, {5.44736842105, 0.301102026839}, {6.05263157895, 0.323727812707}, {6.65789473684, 0.349264886298}, {7.26315789474, 0.377160090308}, {7.86842105263, 0.40665313007}, {8.47368421053, 0.43680573693}, {9.07894736842, 0.466558623071}, {9.68421052632, 0.494811102512}, {10.2894736842, 0.520512409617}, {10.8947368421, 0.54275013704}, {11.5, 0.560821198393}},
    {{0.336842105263, 0}, {0.0, 0.307858119948}, {0.605263157895, 0.305607015579}, {1.21052631579, 0.303873792624}, {1.81578947368, 0.302733920895}, {2.42105263158, 0.302304899526}, {3.02631578947, 0.302777662469}, {3.63157894737, 0.30448109412}, {4.23684210526, 0.308028796882}, {4.84210526316, 0.314701982234}, {5.44736842105, 0.327661177366}, {6.05263157895, 0.352048711733}, {6.65789473684, 0.379256194018}, {7.26315789474, 0.407321384611}, {7.86842105263, 0.435783697171}, {8.47368421053, 0.464179625698}, {9.07894736842, 0.492050846803}, {9.68421052632, 0.518952082082}, {10.2894736842, 0.544458452706}, {10.8947368421, 0.568172088023}, {11.5, 0.589727791862}},
    {{0.378947368421, 0}, {0.0, 0.344281164912}, {0.605263157895, 0.341664097356}, {1.21052631579, 0.339563481569}, {1.81578947368, 0.338042469482}, {2.42105263158, 0.337197681372}, {3.02631578947, 0.337182566886}, {3.63157894737, 0.338253515648}, {4.23684210526, 0.340869413593}, {4.84210526316, 0.345933452435}, {5.44736842105, 0.35548308617}, {6.05263157895, 0.375163213966}, {6.65789473684, 0.406657495893}, {7.26315789474, 0.436408453761}, {7.86842105263, 0.464637260768}, {8.47368421053, 0.491696243936}, {9.07894736842, 0.517856034388}, {9.68421052632, 0.543330166822}, {10.2894736842, 0.56829227083}, {10.8947368421, 0.592888401582}, {11.5, 0.61724621677}},
    {{0.421052631579, 0}, {0.0, 0.380238976321}, {0.605263157895, 0.377250474209}, {1.21052631579, 0.374784851279}, {1.81578947368, 0.3728963891}, {2.42105263158, 0.371666453254}, {3.02631578947, 0.371221097886}, {3.63157894737, 0.371764454033}, {4.23684210526, 0.373647217313}, {4.84210526316, 0.377522021105}, {5.44736842105, 0.384746439478}, {6.05263157895, 0.398639793007}, {6.65789473684, 0.428652542332}, {7.26315789474, 0.463197731471}, {7.86842105263, 0.492587076233}, {8.47368421053, 0.518982059673}, {9.07894736842, 0.543752049033}, {9.68421052632, 0.567855742902}, {10.2894736842, 0.59205153521}, {10.8947368421, 0.617027130666}, {11.5, 0.643496903366}},
    {{0.463157894737, 0}, {0.0, 0.415751419799}, {0.605263157895, 0.412385687462}, {1.21052631579, 0.409556929369}, {1.81578947368, 0.407313944274}, {2.42105263158, 0.405728040157}, {3.02631578947, 0.404906611302}, {3.63157894737, 0.40501787883}, {4.23684210526, 0.406339145755}, {4.84210526316, 0.409359326511}, {5.44736842105, 0.415022327995}, {6.05263157895, 0.425396809272}, {6.65789473684, 0.445918841527}, {7.26315789474, 0.48542850674}, {7.86842105263, 0.518713379918}, {8.47368421053, 0.545586782126}, {9.07894736842, 0.569503689001}, {9.68421052632, 0.592439341107}, {10.2894736842, 0.615773994818}, {10.8947368421, 0.640725296322}, {11.5, 0.66863483184}},
    {{0.505263157895, 0}, {0.0, 0.450857884727}, {0.605263157895, 0.44710713899}, {1.21052631579, 0.443914412461}, {1.81578947368, 0.441326300021}, {2.42105263158, 0.439408771782}, {3.02631578947, 0.438257967907}, {3.63157894737, 0.43801909408}, {4.23684210526, 0.438921485978}, {4.84210526316, 0.441348541161}, {5.44736842105, 0.44599047335}, {6.05263157895, 0.454219630673}, {6.65789473684, 0.469165666461}, {7.26315789474, 0.499363559625}, {7.86842105263, 0.541518705086}, {8.47368421053, 0.570922517094}, {9.07894736842, 0.594849910021}, {9.68421052632, 0.616990131638}, {10.2894736842, 0.639497229208}, {10.8947368421, 0.664125539019}, {11.5, 0.692844986185}},
    {{0.547368421053, 0}, {0.0, 0.485620447755}, {0.605263157895, 0.481472821564}, {1.21052631579, 0.477909858738}, {1.81578947368, 0.474979099226}, {2.42105263158, 0.472745427384}, {3.02631578947, 0.471299934487}, {3.63157894737, 0.470774814533}, {4.23684210526, 0.471369672388}, {4.84210526316, 0.473400858129}, {5.44736842105, 0.477401070023}, {6.05263157895, 0.48433995965}, {6.65789473684, 0.496173557169}, {7.26315789474, 0.517419332341}, {7.86842105263, 0.558375121556}, {8.47368421053, 0.594165098714}, {9.07894736842, 0.619487530028}, {9.68421052632, 0.641414339131}, {10.2894736842, 0.663258405839}, {10.8947368421, 0.687372604142}, {11.5, 0.71633597816}},
    {{0.589473684211, 0}, {0.0, 0.520126008198}, {0.605263157895, 0.515563208933}, {1.21052631579, 0.511615230696}, {1.81578947368, 0.508333580231}, {2.42105263158, 0.50578590617}, {3.02631578947, 0.504063476099}, {3.63157894737, 0.503293234614}, {4.23684210526, 0.503658070322}, {4.84210526316, 0.505432593286}, {5.44736842105, 0.509050129618}, {6.05263157895, 0.515237509324}, {6.65789473684, 0.525311181318}, {7.26315789474, 0.541899627987}, {7.86842105263, 0.570881190859}, {8.47368421053, 0.614088648264}, {9.07894736842, 0.643049264626}, {9.68421052632, 0.66561352097}, {10.2894736842, 0.687094044957}, {10.8947368421, 0.710609832289}, {11.5, 0.739332275552}},
    {{0.631578947368, 0}, {0.0, 0.554486724043}, {0.605263157895, 0.549481782831}, {1.21052631579, 0.54512241677}, {1.81578947368, 0.541467005481}, {2.42105263158, 0.538589512159}, {3.02631578947, 0.536585906705}, {3.63157894737, 0.535584088217}, {4.23684210526, 0.535759741102}, {4.84210526316, 0.537362688651}, {5.44736842105, 0.540762757783}, {6.05263157895, 0.54653406012}, {6.65789473684, 0.555620148331}, {7.26315789474, 0.569682613548}, {7.86842105263, 0.591894299483}, {8.47368421053, 0.628748765989}, {9.07894736842, 0.665072936264}, {9.68421052632, 0.689482643523}, {10.2894736842, 0.71103979495}, {10.8947368421, 0.73397582023}, {11.5, 0.762065608192}},
    {{0.673684210526, 0}, {0.0, 0.588838105689}, {0.605263157895, 0.583353689982}, {1.21052631579, 0.578542369965}, {1.81578947368, 0.574472178557}, {2.42105263158, 0.57122674642}, {3.02631578947, 0.56891086662}, {3.63157894737, 0.56765869742}, {4.23684210526, 0.567646183954}, {4.84210526316, 0.569110450545}, {5.44736842105, 0.572381143003}, {6.05263157895, 0.577933126682}, {6.65789473684, 0.586479153725}, {7.26315789474, 0.599141080777}, {7.86842105263, 0.617782167544}, {8.47368421053, 0.645668337171}, {9.07894736842, 0.684957310964}, {9.68421052632, 0.712907878642}, {10.2894736842, 0.73513022083}, {10.8947368421, 0.757601401371}, {11.5, 0.784766197678}},
    {{0.715789473684, 0}, {0.0, 0.623334319432}, {0.605263157895, 0.617322153216}, {1.21052631579, 0.612002582034}, {1.81578947368, 0.607455869891}, {2.42105263158, 0.603778517378}, {3.02631578947, 0.60108809906}, {3.63157894737, 0.599530008123}, {4.23684210526, 0.599287049406}, {4.84210526316, 0.600593384845}, {5.44736842105, 0.603755295995}, {6.05263157895, 0.609182908571}, {6.65789473684, 0.617440025336}, {7.26315789474, 0.629324626057}, {7.86842105263, 0.646002103424}, {8.47368421053, 0.669227543879}, {9.07894736842, 0.701696589973}, {9.68421052632, 0.735764025897}, {10.2894736842, 0.759398608228}, {10.8947368421, 0.781607067334}, {11.5, 0.807654454013}},
    {{0.757894736842, 0}, {0.0, 0.658140633867}, {0.605263157895, 0.651542523377}, {1.21052631579, 0.645642773655}, {1.81578947368, 0.640536056083}, {2.42105263158, 0.636334713012}, {3.02631578947, 0.633173004856}, {3.63157894737, 0.631212610695}, {4.23684210526, 0.630649817914}, {4.84210526316, 0.631725005442}, {5.44736842105, 0.634735260235}, {6.05263157895, 0.640051284621}, {6.65789473684, 0.648140152901}, {7.26315789474, 0.65959588902}, {7.86842105263, 0.67517995569}, {8.47368421053, 0.695872504349}, {9.07894736842, 0.722929169486}, {9.68421052632, 0.757911443502}, {10.2894736842, 0.783876784955}, {10.8947368421, 0.806100917665}, {11.5, 0.830933706697}},
    {{0.8, 0}, {0.0, 0.693423469819}, {0.605263157895, 0.686174232569}, {1.21052631579, 0.679608916142}, {1.81578947368, 0.673838001467}, {2.42105263158, 0.668992126962}, {3.02631578947, 0.665225964162}, {3.63157894737, 0.662722743774}, {4.23684210526, 0.661699436285}, {4.84210526316, 0.662412493414}, {5.44736842105, 0.665163871676}, {6.05263157895, 0.670306729212}, {6.65789473684, 0.678249622985}, {7.26315789474, 0.689457110254}, {7.86842105263, 0.704443221378}, {8.47368421053, 0.723752190042}, {9.07894736842, 0.747918159573}, {9.68421052632, 0.777392972218}, {10.2894736842, 0.808594961837}, {10.8947368421, 0.831177186305}, {11.5, 0.854784402794}}
};

#endif