#include "Copter.h"
#include <stdio.h>
#include <../libraries/AP_RangeFinder/AP_RangeFinder_ETRI.h>

#if MODE_CNDN_ENABLED == ENABLED

#define USE_ETRI DISABLED

int degNE(const Vector2f& pp)
{
    Vector2f npos = pp.normalized();
    Vector2f nort(1, 0);
    int rd = (int)(nort.angle(npos) * 180 / M_PI);
    if (npos.y<0) rd = 360 - rd;
    return rd;
}

int degNE(const Vector2f& p1, const Vector2f& p2)
{
    return degNE(p1-p2);
}

Vector3f locNEU(float latf, float lngf, float altf)
{
    Vector3f pos;
    int32_t lat = latf * 1e7f;
    int32_t lng = lngf * 1e7f;
    const Location lc {lat,lng,(int)(altf*100),Location::AltFrame::ABSOLUTE,};
    if (lc.check_latlng() && lc.get_vector_from_origin_NEU(pos))
        return pos;
    return pos;
}

bool inside(const CNAREA& area, const Vector2f& cp)
{
    int cross = 0;
    const Vector2f *vp;
    vp = (const Vector2f *)area.pos;

    for (uint16_t i=0; i<4; i++)
    {
        int j=(i + 1) % 4;
        if ((vp[i].y > cp.y) != (vp[j].y > cp.y))
        {
            double aX = (vp[j].x-vp[i].x)*(cp.y-vp[i].y)/(vp[j].y-vp[i].y)+vp[i].x;
            if (cp.x < aX)
                cross ++;
        }
    }
    return (cross % 2) > 0;
}

bool inside(const CNAREA& area, const Location& loc)
{
    return inside(area, Vector2f(loc.lat*1e-7, loc.lng*1e-7));
}

const AP_Param::GroupInfo ModeCNDN::var_info[] = {
    // @Param: METHOD
    // @DisplayName: Mode using method
    // @Description: Mode using method of CNDN & ETRI Mission computer
    // @Values: 0: Disable, 1: All enable, 2: Take picture only, 2: Edge follow only, 3: Take picture after Edge following
    // @User: Standard
    AP_GROUPINFO_FLAGS("METHOD", 0, ModeCNDN, _method, 3, AP_PARAM_FLAG_ENABLE),

    // @Param: TAKE_ALT
    // @DisplayName: Take picture altitute
    // @Description: Altitute of take picture
    // @Units: cm
    // @Range: 2000 4000
    // @User: Standard
    AP_GROUPINFO("TAKE_ALT", 1, ModeCNDN, _take_alt_cm, 300),

    // @Param: MISSION_ALT
    // @DisplayName: Mission altitute
    // @Description: Altitute of mission planning
    // @Units: cm
    // @Range: 200 1000
    // @User: Standard
    AP_GROUPINFO("MISSION_ALT", 2, ModeCNDN, _mission_alt_cm, 300),

    // @Param: SPRAY_WIDTH
    // @DisplayName: Spray width
    // @Description: Mission planning width of spraying
    // @Units: cm
    // @Range: 3000 8000
    // @User: Standard
    AP_GROUPINFO("SPRAY_WIDTH", 3, ModeCNDN, _spray_width_cm, 400),

    // @Param: ACC_XY
    // @DisplayName: Acceleration xy
    // @Description: ground speed acceleration
    // @Units: cms
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("ACC_XY", 4, ModeCNDN, _acc_xy_cms, 200),

    // @Param: SPD_XY
    // @DisplayName: Speed xy
    // @Description: ground speed
    // @Units: cms
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("SPD_XY", 5, ModeCNDN, _spd_xy_cmss, 400),

    // @Param: SPD_UP
    // @DisplayName: Z Speed Up
    // @Description: z speed for up
    // @Units: cms
    // @Range: 50 1000
    // @User: Standard
    AP_GROUPINFO("SPD_UP", 6, ModeCNDN, _spd_up_cmss, 150),

    // @Param: SPD_DN
    // @DisplayName: Z Speed Down
    // @Description: z speed for down
    // @Units: cms
    // @Range: 30 500
    // @User: Standard
    AP_GROUPINFO("SPD_DN", 7, ModeCNDN, _spd_dn_cmss, 150),

    // @Param: SPD_EDGE
    // @DisplayName: Edge Speed
    // @Description: ground Speed xy for Edge
    // @Units: cms
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("SPD_EDGE", 8, ModeCNDN, _spd_eg_cmss, 300),

    // @Param: DIS_EDGE
    // @DisplayName: Distance Edge
    // @Description: Distance from Edge
    // @Units: cms
    // @Range: 100 800
    // @User: Standard
    AP_GROUPINFO("DIS_EDGE", 9, ModeCNDN, _dst_eg_cm, 350),

    AP_GROUPEND
};

ModeCNDN::ModeCNDN()
{
    AP_Param::setup_object_defaults(this, var_info);

#if defined(SIM_LOCATION)
    if (vecAreas.empty())
    {
        // area1
        vecAreas.push_back({pos:{Vector2f(36.11120270f,127.5232490f),Vector2f(36.11129479f,127.5240063f),Vector2f(36.11109010f,127.5240575f),Vector2f(36.11099559f,127.5232995f)}});
        // area2
        vecAreas.push_back({pos:{Vector2f(36.11093780f,127.5233028f),Vector2f(36.11103150f,127.5240753f),Vector2f(36.11094320f,127.5240981f),Vector2f(36.11083540f,127.5233082f)}});
        // area3
        vecAreas.push_back({pos:{Vector2f(36.11079300f,127.5233048f),Vector2f(36.11090030f,127.5240907f),Vector2f(36.11081250f,127.5241095f),Vector2f(36.11069440f,127.5232901f)}});
        // area4
        vecAreas.push_back({pos:{Vector2f(36.11125500f,127.5232401f),Vector2f(36.11146870f,127.5231922f),Vector2f(36.11156560f,127.5239358f),Vector2f(36.11135000f,127.5239854f)}});
        // area 0
        vecAreas.push_back({pos:{Vector2f(37.28421031f,126.87357650f),Vector2f(37.28363457f,126.87271328f),Vector2f(37.28391077f,126.87242739f),Vector2f(37.28448638f,126.87328855f)}});

        vecAreas.push_back({pos:{Vector2f(35.80339634f,126.82592990f),Vector2f(35.80315059f,126.82594349f),Vector2f(35.80311112f,126.82478148f),Vector2f(35.80335752f,126.82477618f)}});
        vecAreas.push_back({pos:{Vector2f(35.80315059f,126.82594349f),Vector2f(35.80282823f,126.82596146f),Vector2f(35.80278777f,126.82479282f),Vector2f(35.80311112f,126.82478148f)}});
        vecAreas.push_back({pos:{Vector2f(35.80327455f,126.82971530f),Vector2f(35.80294696f,126.82973094f),Vector2f(35.80290849f,126.82853983f),Vector2f(35.80323985f,126.82852384f)}});
        vecAreas.push_back({pos:{Vector2f(35.80360574f,126.82970043f),Vector2f(35.80327455f,126.82971530f),Vector2f(35.80323985f,126.82852384f),Vector2f(35.80356608f,126.82850632f)}});
        vecAreas.push_back({pos:{Vector2f(35.80393368f,126.82968324f),Vector2f(35.80360574f,126.82970043f),Vector2f(35.80356608f,126.82850632f),Vector2f(35.80389637f,126.82848945f)}});
        vecAreas.push_back({pos:{Vector2f(35.80422108f,126.82841960f),Vector2f(35.80389043f,126.82843558f),Vector2f(35.80385109f,126.82727997f),Vector2f(35.80417976f,126.82726344f)}});
        vecAreas.push_back({pos:{Vector2f(35.80389043f,126.82843558f),Vector2f(35.80356564f,126.82845156f),Vector2f(35.80352729f,126.82729551f),Vector2f(35.80385109f,126.82727997f)}});
        vecAreas.push_back({pos:{Vector2f(35.80425739f,126.82966716f),Vector2f(35.80393368f,126.82968324f),Vector2f(35.80389637f,126.82848945f),Vector2f(35.80422477f,126.82847447f)}});
        vecAreas.push_back({pos:{Vector2f(35.80579363f,126.82964936f),Vector2f(35.80584169f,126.82964691f),Vector2f(35.80590569f,126.82978661f),Vector2f(35.80590830f,126.82986374f)}});
        vecAreas.push_back({pos:{Vector2f(35.80356564f,126.82845156f),Vector2f(35.80323986f,126.82846775f),Vector2f(35.80320106f,126.82730983f),Vector2f(35.80352729f,126.82729551f)}});
        vecAreas.push_back({pos:{Vector2f(35.80323986f,126.82846775f),Vector2f(35.80290390f,126.82848319f),Vector2f(35.80287059f,126.82732493f),Vector2f(35.80320106f,126.82730983f)}});
        vecAreas.push_back({pos:{Vector2f(35.80090991f,126.83743400f),Vector2f(35.80085136f,126.83741872f),Vector2f(35.80089072f,126.83732246f),Vector2f(35.80094745f,126.83736131f)}});
        vecAreas.push_back({pos:{Vector2f(35.80458083f,126.82965164f),Vector2f(35.80425739f,126.82966716f),Vector2f(35.80422477f,126.82847447f),Vector2f(35.80454559f,126.82845784f)}});
        vecAreas.push_back({pos:{Vector2f(35.80490850f,126.82963611f),Vector2f(35.80458083f,126.82965164f),Vector2f(35.80454559f,126.82845784f),Vector2f(35.80487470f,126.82843810f)}});
        vecAreas.push_back({pos:{Vector2f(35.79730853f,126.82366096f),Vector2f(35.79729566f,126.82324614f),Vector2f(35.79822514f,126.82319616f),Vector2f(35.79823621f,126.82361144f)}});
        vecAreas.push_back({pos:{Vector2f(35.79732267f,126.82407987f),Vector2f(35.79730853f,126.82366096f),Vector2f(35.79823621f,126.82361144f),Vector2f(35.79825233f,126.82402636f)}});
        vecAreas.push_back({pos:{Vector2f(35.79826394f,126.82444362f),Vector2f(35.79734031f,126.82449391f),Vector2f(35.79732267f,126.82407987f),Vector2f(35.79825233f,126.82402636f)}});
        vecAreas.push_back({pos:{Vector2f(35.79827591f,126.82486166f),Vector2f(35.79735300f,126.82491027f),Vector2f(35.79734031f,126.82449391f),Vector2f(35.79826394f,126.82444362f)}});
        vecAreas.push_back({pos:{Vector2f(35.80404990f,126.82339339f),Vector2f(35.80404405f,126.82321928f),Vector2f(35.80442916f,126.82320823f),Vector2f(35.80446748f,126.82337861f)}});
        vecAreas.push_back({pos:{Vector2f(35.80215367f,126.84207805f),Vector2f(35.80225192f,126.84199504f),Vector2f(35.80226535f,126.84216223f),Vector2f(35.80219087f,126.84218603f)}});
        vecAreas.push_back({pos:{Vector2f(35.80208702f,126.84225328f),Vector2f(35.80205164f,126.84217457f),Vector2f(35.80215367f,126.84207805f),Vector2f(35.80219087f,126.84218603f)}});
        vecAreas.push_back({pos:{Vector2f(35.80208702f,126.84225328f),Vector2f(35.80199134f,126.84232561f),Vector2f(35.80194716f,126.84225283f),Vector2f(35.80205164f,126.84217457f)}});
        vecAreas.push_back({pos:{Vector2f(35.80165427f,126.84231541f),Vector2f(35.80170618f,126.84225384f),Vector2f(35.80178043f,126.84235100f),Vector2f(35.80172233f,126.84242459f)}});
        vecAreas.push_back({pos:{Vector2f(35.80116285f,126.84081186f),Vector2f(35.80115622f,126.84073997f),Vector2f(35.80120998f,126.84078659f),Vector2f(35.80118910f,126.84082074f)}});
        vecAreas.push_back({pos:{Vector2f(35.80203439f,126.84236528f),Vector2f(35.80208996f,126.84229424f),Vector2f(35.80215339f,126.84241766f),Vector2f(35.80209700f,126.84244139f)}});
        vecAreas.push_back({pos:{Vector2f(35.79828941f,126.82527692f),Vector2f(35.79736993f,126.82532697f),Vector2f(35.79735300f,126.82491027f),Vector2f(35.79827591f,126.82486166f)}});
        vecAreas.push_back({pos:{Vector2f(35.79830200f,126.82569120f),Vector2f(35.79738423f,126.82574355f),Vector2f(35.79736993f,126.82532697f),Vector2f(35.79828941f,126.82527692f)}});
        vecAreas.push_back({pos:{Vector2f(35.79744206f,126.82745736f),Vector2f(35.79742785f,126.82704132f),Vector2f(35.79834752f,126.82699362f),Vector2f(35.79836001f,126.82740701f)}});
        vecAreas.push_back({pos:{Vector2f(35.79745690f,126.82787085f),Vector2f(35.79744206f,126.82745736f),Vector2f(35.79836001f,126.82740701f),Vector2f(35.79837422f,126.82782150f)}});
        vecAreas.push_back({pos:{Vector2f(35.79838581f,126.82823666f),Vector2f(35.79747174f,126.82828445f),Vector2f(35.79745690f,126.82787085f),Vector2f(35.79837422f,126.82782150f)}});
        vecAreas.push_back({pos:{Vector2f(35.79748599f,126.82866630f),Vector2f(35.79747174f,126.82828445f),Vector2f(35.79838581f,126.82823666f),Vector2f(35.79840303f,126.82861342f)}});
        vecAreas.push_back({pos:{Vector2f(35.79842640f,126.82944828f),Vector2f(35.79751070f,126.82949583f),Vector2f(35.79749541f,126.82908079f),Vector2f(35.79841463f,126.82903323f)}});
        vecAreas.push_back({pos:{Vector2f(35.79844078f,126.82986288f),Vector2f(35.79752300f,126.82990833f),Vector2f(35.79751070f,126.82949583f),Vector2f(35.79842640f,126.82944828f)}});
        vecAreas.push_back({pos:{Vector2f(35.79759854f,126.83194591f),Vector2f(35.79758354f,126.83154072f),Vector2f(35.79851006f,126.83149648f),Vector2f(35.79852406f,126.83190235f)}});
        vecAreas.push_back({pos:{Vector2f(35.79761127f,126.83234856f),Vector2f(35.79759854f,126.83194591f),Vector2f(35.79852406f,126.83190235f),Vector2f(35.79853941f,126.83230124f)}});
        vecAreas.push_back({pos:{Vector2f(35.79762273f,126.83274458f),Vector2f(35.79761127f,126.83234856f),Vector2f(35.79853941f,126.83230124f),Vector2f(35.79855556f,126.83269969f)}});
        vecAreas.push_back({pos:{Vector2f(35.79856757f,126.83310201f),Vector2f(35.79763844f,126.83315132f),Vector2f(35.79762273f,126.83274458f),Vector2f(35.79855556f,126.83269969f)}});
        vecAreas.push_back({pos:{Vector2f(35.79765234f,126.83355364f),Vector2f(35.79763844f,126.83315132f),Vector2f(35.79856757f,126.83310201f),Vector2f(35.79858444f,126.83350389f)}});
        vecAreas.push_back({pos:{Vector2f(35.79766588f,126.83395353f),Vector2f(35.79765234f,126.83355364f),Vector2f(35.79858444f,126.83350389f),Vector2f(35.79859555f,126.83390390f)}});
        vecAreas.push_back({pos:{Vector2f(35.79861206f,126.83430765f),Vector2f(35.79767897f,126.83435640f),Vector2f(35.79766588f,126.83395353f),Vector2f(35.79859555f,126.83390390f)}});
        vecAreas.push_back({pos:{Vector2f(35.79769386f,126.83476071f),Vector2f(35.79768906f,126.83455418f),Vector2f(35.79861829f,126.83450965f),Vector2f(35.79862632f,126.83471053f)}});
        vecAreas.push_back({pos:{Vector2f(35.79770964f,126.83515794f),Vector2f(35.79769386f,126.83476071f),Vector2f(35.79862632f,126.83471053f),Vector2f(35.79863895f,126.83511197f)}});
        vecAreas.push_back({pos:{Vector2f(35.79865014f,126.83551574f),Vector2f(35.79772308f,126.83555971f),Vector2f(35.79770964f,126.83515794f),Vector2f(35.79863895f,126.83511197f)}});
        vecAreas.push_back({pos:{Vector2f(35.79773545f,126.83596590f),Vector2f(35.79772308f,126.83555971f),Vector2f(35.79865014f,126.83551574f),Vector2f(35.79866575f,126.83591652f)}});
        vecAreas.push_back({pos:{Vector2f(35.79867756f,126.83631409f),Vector2f(35.79775015f,126.83636634f),Vector2f(35.79773545f,126.83596590f),Vector2f(35.79866575f,126.83591652f)}});
        vecAreas.push_back({pos:{Vector2f(35.79777527f,126.83720351f),Vector2f(35.79776210f,126.83679887f),Vector2f(35.79867123f,126.83675772f),Vector2f(35.79868800f,126.83715650f)}});
        vecAreas.push_back({pos:{Vector2f(35.79869738f,126.83756182f),Vector2f(35.79778961f,126.83760893f),Vector2f(35.79777527f,126.83720351f),Vector2f(35.79868800f,126.83715650f)}});
        vecAreas.push_back({pos:{Vector2f(35.79871208f,126.83796702f),Vector2f(35.79780368f,126.83801147f),Vector2f(35.79778961f,126.83760893f),Vector2f(35.79869738f,126.83756182f)}});
        vecAreas.push_back({pos:{Vector2f(35.79872507f,126.83836846f),Vector2f(35.79781422f,126.83841115f),Vector2f(35.79780368f,126.83801147f),Vector2f(35.79871208f,126.83796702f)}});
        vecAreas.push_back({pos:{Vector2f(35.79784505f,126.83921523f),Vector2f(35.79782990f,126.83880804f),Vector2f(35.79873868f,126.83876747f),Vector2f(35.79875049f,126.83917190f)}});
        vecAreas.push_back({pos:{Vector2f(35.79876076f,126.83957092f),Vector2f(35.79785876f,126.83961977f),Vector2f(35.79784505f,126.83921523f),Vector2f(35.79875049f,126.83917190f)}});
        vecAreas.push_back({pos:{Vector2f(35.79877291f,126.83996351f),Vector2f(35.79787560f,126.84001301f),Vector2f(35.79785876f,126.83961977f),Vector2f(35.79876076f,126.83957092f)}});
        vecAreas.push_back({pos:{Vector2f(35.80305300f,126.84092348f),Vector2f(35.80308669f,126.84089738f),Vector2f(35.80315836f,126.84102359f),Vector2f(35.80310341f,126.84100896f)}});
        vecAreas.push_back({pos:{Vector2f(35.80815592f,126.83444540f),Vector2f(35.80768218f,126.83435655f),Vector2f(35.80763809f,126.83427964f),Vector2f(35.80791390f,126.83403831f)}});
        vecAreas.push_back({pos:{Vector2f(35.80499851f,126.83209094f),Vector2f(35.80467453f,126.83210701f),Vector2f(35.80463339f,126.83093424f),Vector2f(35.80496044f,126.83091772f)}});
        vecAreas.push_back({pos:{Vector2f(35.80467453f,126.83210701f),Vector2f(35.80434685f,126.83212275f),Vector2f(35.80430392f,126.83095210f),Vector2f(35.80463339f,126.83093424f)}});
        vecAreas.push_back({pos:{Vector2f(35.80434685f,126.83212275f),Vector2f(35.80401441f,126.83214127f),Vector2f(35.80397525f,126.83096707f),Vector2f(35.80430392f,126.83095210f)}});
        vecAreas.push_back({pos:{Vector2f(35.80401441f,126.83214127f),Vector2f(35.80368997f,126.83215656f),Vector2f(35.80364893f,126.83098403f),Vector2f(35.80397525f,126.83096707f)}});
        vecAreas.push_back({pos:{Vector2f(35.80368997f,126.83215656f),Vector2f(35.80335951f,126.83217320f),Vector2f(35.80332405f,126.83100099f),Vector2f(35.80364893f,126.83098403f)}});
        vecAreas.push_back({pos:{Vector2f(35.80335951f,126.83217320f),Vector2f(35.80303553f,126.83218992f),Vector2f(35.80299611f,126.83101640f),Vector2f(35.80332405f,126.83100099f)}});
        vecAreas.push_back({pos:{Vector2f(35.80502579f,126.83290551f),Vector2f(35.80470848f,126.83317812f),Vector2f(35.80467102f,126.83217661f),Vector2f(35.80499843f,126.83216264f)}});
        vecAreas.push_back({pos:{Vector2f(35.80250332f,126.82806930f),Vector2f(35.80154707f,126.82811585f),Vector2f(35.80153424f,126.82771738f),Vector2f(35.80249094f,126.82767182f)}});
        vecAreas.push_back({pos:{Vector2f(35.80251678f,126.82846523f),Vector2f(35.80156071f,126.82851576f),Vector2f(35.80154707f,126.82811585f),Vector2f(35.80250332f,126.82806930f)}});
        vecAreas.push_back({pos:{Vector2f(35.80252972f,126.82887388f),Vector2f(35.80157463f,126.82892108f),Vector2f(35.80156071f,126.82851576f),Vector2f(35.80251678f,126.82846523f)}});
        vecAreas.push_back({pos:{Vector2f(35.80254363f,126.82927489f),Vector2f(35.80158773f,126.82931966f),Vector2f(35.80157463f,126.82892108f),Vector2f(35.80252972f,126.82887388f)}});
        vecAreas.push_back({pos:{Vector2f(35.80255620f,126.82968056f),Vector2f(35.80160146f,126.82972167f),Vector2f(35.80158773f,126.82931966f),Vector2f(35.80254363f,126.82927489f)}});
        vecAreas.push_back({pos:{Vector2f(35.80257101f,126.83008135f),Vector2f(35.80161467f,126.83013042f),Vector2f(35.80160146f,126.82972167f),Vector2f(35.80255620f,126.82968056f)}});
        vecAreas.push_back({pos:{Vector2f(35.80258465f,126.83048281f),Vector2f(35.80162893f,126.83052900f),Vector2f(35.80161467f,126.83013042f),Vector2f(35.80257101f,126.83008135f)}});
        vecAreas.push_back({pos:{Vector2f(35.80376145f,126.82416885f),Vector2f(35.80372292f,126.82411276f),Vector2f(35.80376029f,126.82407593f),Vector2f(35.80380275f,126.82413560f)}});
        vecAreas.push_back({pos:{Vector2f(35.80259891f,126.83088007f),Vector2f(35.80164257f,126.83092769f),Vector2f(35.80162893f,126.83052900f),Vector2f(35.80258465f,126.83048281f)}});
        vecAreas.push_back({pos:{Vector2f(35.80261426f,126.83127831f),Vector2f(35.80165585f,126.83133169f),Vector2f(35.80164257f,126.83092769f),Vector2f(35.80259891f,126.83088007f)}});
        vecAreas.push_back({pos:{Vector2f(35.80262736f,126.83168531f),Vector2f(35.80166777f,126.83173525f),Vector2f(35.80165585f,126.83133169f),Vector2f(35.80261426f,126.83127831f)}});
        vecAreas.push_back({pos:{Vector2f(35.80263919f,126.83208356f),Vector2f(35.80168149f,126.83213483f),Vector2f(35.80166777f,126.83173525f),Vector2f(35.80262736f,126.83168531f)}});
        vecAreas.push_back({pos:{Vector2f(35.80383950f,126.82214963f),Vector2f(35.80367962f,126.82215662f),Vector2f(35.80363431f,126.82096430f),Vector2f(35.80379418f,126.82095763f)}});
        vecAreas.push_back({pos:{Vector2f(35.80367962f,126.82215662f),Vector2f(35.80334969f,126.82216887f),Vector2f(35.80330456f,126.82098120f),Vector2f(35.80363431f,126.82096430f)}});
        vecAreas.push_back({pos:{Vector2f(35.80334969f,126.82216887f),Vector2f(35.80301877f,126.82218300f),Vector2f(35.80297833f,126.82099433f),Vector2f(35.80330456f,126.82098120f)}});
        vecAreas.push_back({pos:{Vector2f(35.80301877f,126.82218300f),Vector2f(35.80269362f,126.82219944f),Vector2f(35.80264597f,126.82100869f),Vector2f(35.80297833f,126.82099433f)}});
        vecAreas.push_back({pos:{Vector2f(35.80108130f,126.83857123f),Vector2f(35.80111480f,126.83838765f),Vector2f(35.80117923f,126.83838558f),Vector2f(35.80122786f,126.83847370f)}});
        vecAreas.push_back({pos:{Vector2f(35.80147141f,126.83961038f),Vector2f(35.80127552f,126.83926255f),Vector2f(35.80154120f,126.83902649f),Vector2f(35.80173820f,126.83937511f)}});
        vecAreas.push_back({pos:{Vector2f(35.80339762f,126.82343011f),Vector2f(35.80322532f,126.82344233f),Vector2f(35.80318993f,126.82224867f),Vector2f(35.80335341f,126.82224376f)}});
        vecAreas.push_back({pos:{Vector2f(35.80322532f,126.82344233f),Vector2f(35.80306924f,126.82345341f),Vector2f(35.80302636f,126.82225423f),Vector2f(35.80318993f,126.82224867f)}});
        vecAreas.push_back({pos:{Vector2f(35.80432563f,126.82298388f),Vector2f(35.80438033f,126.82298197f),Vector2f(35.80442916f,126.82320823f),Vector2f(35.80437224f,126.82320986f)}});
        vecAreas.push_back({pos:{Vector2f(35.80561365f,126.83084496f),Vector2f(35.80528571f,126.83086060f),Vector2f(35.80524349f,126.82969093f),Vector2f(35.80557224f,126.82967517f)}});
        vecAreas.push_back({pos:{Vector2f(35.80528571f,126.83086060f),Vector2f(35.80495975f,126.83087767f),Vector2f(35.80491825f,126.82970789f),Vector2f(35.80524349f,126.82969093f)}});
        vecAreas.push_back({pos:{Vector2f(35.80495975f,126.83087767f),Vector2f(35.80463388f,126.83089585f),Vector2f(35.80459291f,126.82972153f),Vector2f(35.80491825f,126.82970789f)}});
        vecAreas.push_back({pos:{Vector2f(35.80463388f,126.83089585f),Vector2f(35.80430359f,126.83091116f),Vector2f(35.80426578f,126.82973828f),Vector2f(35.80459291f,126.82972153f)}});
        vecAreas.push_back({pos:{Vector2f(35.80430359f,126.83091116f),Vector2f(35.80397267f,126.83092714f),Vector2f(35.80393676f,126.82975536f),Vector2f(35.80426578f,126.82973828f)}});
        vecAreas.push_back({pos:{Vector2f(35.80397267f,126.83092714f),Vector2f(35.80364256f,126.83094034f),Vector2f(35.80360629f,126.82977289f),Vector2f(35.80393676f,126.82975536f)}});
        vecAreas.push_back({pos:{Vector2f(35.80364256f,126.83094034f),Vector2f(35.80332101f,126.83095497f),Vector2f(35.80327960f,126.82978410f),Vector2f(35.80360629f,126.82977289f)}});
        vecAreas.push_back({pos:{Vector2f(35.80332101f,126.83095497f),Vector2f(35.80299064f,126.83097161f),Vector2f(35.80295121f,126.82980163f),Vector2f(35.80327960f,126.82978410f)}});
        vecAreas.push_back({pos:{Vector2f(35.80565196f,126.83205692f),Vector2f(35.80532807f,126.83207265f),Vector2f(35.80528811f,126.83090220f),Vector2f(35.80561542f,126.83088590f)}});
        vecAreas.push_back({pos:{Vector2f(35.80532807f,126.83207265f),Vector2f(35.80499851f,126.83209094f),Vector2f(35.80496044f,126.83091772f),Vector2f(35.80528811f,126.83090220f)}});
        vecAreas.push_back({pos:{Vector2f(35.80524624f,126.83362614f),Vector2f(35.80513282f,126.83372484f),Vector2f(35.80484735f,126.83362797f),Vector2f(35.80511227f,126.83339829f)}});
        vecAreas.push_back({pos:{Vector2f(35.80787994f,126.83398151f),Vector2f(35.80762017f,126.83421960f),Vector2f(35.80704867f,126.83324086f),Vector2f(35.80731294f,126.83300486f)}});
        vecAreas.push_back({pos:{Vector2f(35.80762017f,126.83421960f),Vector2f(35.80750098f,126.83432086f),Vector2f(35.80693156f,126.83334411f),Vector2f(35.80704867f,126.83324086f)}});
        vecAreas.push_back({pos:{Vector2f(35.80326552f,126.83554213f),Vector2f(35.80316265f,126.83564118f),Vector2f(35.80294721f,126.83527404f),Vector2f(35.80305660f,126.83518141f)}});
        vecAreas.push_back({pos:{Vector2f(35.79622435f,126.84050940f),Vector2f(35.79626707f,126.84017158f),Vector2f(35.79677011f,126.84014424f),Vector2f(35.79678264f,126.84048018f)}});
        vecAreas.push_back({pos:{Vector2f(35.79486465f,126.83901704f),Vector2f(35.79490595f,126.83899787f),Vector2f(35.79491628f,126.83900350f),Vector2f(35.79490123f,126.83907092f)}});
        vecAreas.push_back({pos:{Vector2f(35.79493011f,126.83904236f),Vector2f(35.79493926f,126.83898979f),Vector2f(35.79497484f,126.83895236f),Vector2f(35.79498904f,126.83903241f)}});
        vecAreas.push_back({pos:{Vector2f(35.79489935f,126.83875575f),Vector2f(35.79493113f,126.83873283f),Vector2f(35.79499496f,126.83882706f),Vector2f(35.79496572f,126.83885295f)}});
        vecAreas.push_back({pos:{Vector2f(35.79605452f,126.83907125f),Vector2f(35.79594437f,126.83881229f),Vector2f(35.79597982f,126.83868276f),Vector2f(35.79607728f,126.83906041f)}});
        vecAreas.push_back({pos:{Vector2f(35.79497027f,126.83785797f),Vector2f(35.79494938f,126.83792279f),Vector2f(35.79494335f,126.83790909f),Vector2f(35.79497589f,126.83780791f)}});
        vecAreas.push_back({pos:{Vector2f(35.79757822f,126.83495090f),Vector2f(35.79741673f,126.83496396f),Vector2f(35.79737616f,126.83385059f),Vector2f(35.79753802f,126.83384494f)}});
        vecAreas.push_back({pos:{Vector2f(35.79708041f,126.83498125f),Vector2f(35.79703939f,126.83386778f),Vector2f(35.79737616f,126.83385059f),Vector2f(35.79741673f,126.83496396f)}});
        vecAreas.push_back({pos:{Vector2f(35.79708041f,126.83498125f),Vector2f(35.79674057f,126.83499900f),Vector2f(35.79670288f,126.83388320f),Vector2f(35.79703939f,126.83386778f)}});
        vecAreas.push_back({pos:{Vector2f(35.79615407f,126.83503020f),Vector2f(35.79611540f,126.83391773f),Vector2f(35.79637079f,126.83390326f),Vector2f(35.79640974f,126.83501684f)}});
        vecAreas.push_back({pos:{Vector2f(35.79583739f,126.83504602f),Vector2f(35.79580043f,126.83393377f),Vector2f(35.79611540f,126.83391773f),Vector2f(35.79615407f,126.83503020f)}});
        vecAreas.push_back({pos:{Vector2f(35.79550710f,126.83506242f),Vector2f(35.79546852f,126.83395073f),Vector2f(35.79580043f,126.83393377f),Vector2f(35.79583739f,126.83504602f)}});
        vecAreas.push_back({pos:{Vector2f(35.79550710f,126.83506242f),Vector2f(35.79525279f,126.83507545f),Vector2f(35.79521627f,126.83396177f),Vector2f(35.79546852f,126.83395073f)}});
        vecAreas.push_back({pos:{Vector2f(35.79728649f,126.83613454f),Vector2f(35.79724783f,126.83502460f),Vector2f(35.79757812f,126.83500677f),Vector2f(35.79761831f,126.83611881f)}});
        vecAreas.push_back({pos:{Vector2f(35.79659122f,126.83617116f),Vector2f(35.79655166f,126.83505880f),Vector2f(35.79695810f,126.83503881f),Vector2f(35.79699658f,126.83614986f)}});
        vecAreas.push_back({pos:{Vector2f(35.79619515f,126.83619179f),Vector2f(35.79615585f,126.83507976f),Vector2f(35.79655166f,126.83505880f),Vector2f(35.79659122f,126.83617116f)}});
        vecAreas.push_back({pos:{Vector2f(35.79591470f,126.83620642f),Vector2f(35.79587675f,126.83509461f),Vector2f(35.79615585f,126.83507976f),Vector2f(35.79619515f,126.83619179f)}});
        vecAreas.push_back({pos:{Vector2f(35.79486211f,126.83541488f),Vector2f(35.79454312f,126.83597817f),Vector2f(35.79413165f,126.83561518f),Vector2f(35.79461800f,126.83478993f)}});
        vecAreas.push_back({pos:{Vector2f(35.79461800f,126.83478993f),Vector2f(35.79413165f,126.83561518f),Vector2f(35.79397473f,126.83547756f),Vector2f(35.79453135f,126.83457041f)}});
        vecAreas.push_back({pos:{Vector2f(35.79453135f,126.83457041f),Vector2f(35.79397473f,126.83547756f),Vector2f(35.79370827f,126.83524404f),Vector2f(35.79435763f,126.83414399f)}});
        vecAreas.push_back({pos:{Vector2f(35.79373654f,126.83666525f),Vector2f(35.79345158f,126.83641450f),Vector2f(35.79395987f,126.83555061f),Vector2f(35.79424429f,126.83580090f)}});
        vecAreas.push_back({pos:{Vector2f(35.79345158f,126.83641450f),Vector2f(35.79317221f,126.83616707f),Vector2f(35.79368068f,126.83530207f),Vector2f(35.79395987f,126.83555061f)}});
        vecAreas.push_back({pos:{Vector2f(35.79317221f,126.83616707f),Vector2f(35.79294698f,126.83596599f),Vector2f(35.79345545f,126.83510110f),Vector2f(35.79368068f,126.83530207f)}});
        vecAreas.push_back({pos:{Vector2f(35.79367934f,126.83669081f),Vector2f(35.79312501f,126.83757260f),Vector2f(35.79286838f,126.83734481f),Vector2f(35.79340412f,126.83644613f)}});
        vecAreas.push_back({pos:{Vector2f(35.79260959f,126.83711846f),Vector2f(35.79241369f,126.83694419f),Vector2f(35.79294789f,126.83603822f),Vector2f(35.79314406f,126.83621315f)}});
        vecAreas.push_back({pos:{Vector2f(35.79294789f,126.83603822f),Vector2f(35.79241369f,126.83694419f),Vector2f(35.79214434f,126.83670504f),Vector2f(35.79267880f,126.83579918f)}});
        vecAreas.push_back({pos:{Vector2f(35.79567868f,126.83621996f),Vector2f(35.79564037f,126.83510561f),Vector2f(35.79587675f,126.83509461f),Vector2f(35.79591470f,126.83620642f)}});
        vecAreas.push_back({pos:{Vector2f(35.79541805f,126.83623023f),Vector2f(35.79537524f,126.83511911f),Vector2f(35.79564037f,126.83510561f),Vector2f(35.79567868f,126.83621996f)}});
        vecAreas.push_back({pos:{Vector2f(35.79534430f,126.83621103f),Vector2f(35.79492213f,126.83515047f),Vector2f(35.79537524f,126.83511911f),Vector2f(35.79541805f,126.83623023f)}});
        vecAreas.push_back({pos:{Vector2f(35.79765805f,126.83729491f),Vector2f(35.79733849f,126.83731216f),Vector2f(35.79730019f,126.83620067f),Vector2f(35.79761931f,126.83618552f)}});
        vecAreas.push_back({pos:{Vector2f(35.79733849f,126.83731216f),Vector2f(35.79701343f,126.83732976f),Vector2f(35.79697576f,126.83621617f),Vector2f(35.79730019f,126.83620067f)}});
        vecAreas.push_back({pos:{Vector2f(35.79668873f,126.83734735f),Vector2f(35.79665196f,126.83623155f),Vector2f(35.79697576f,126.83621617f),Vector2f(35.79701343f,126.83732976f)}});
        vecAreas.push_back({pos:{Vector2f(35.79668873f,126.83734735f),Vector2f(35.79636403f,126.83736494f),Vector2f(35.79632870f,126.83624870f),Vector2f(35.79665196f,126.83623155f)}});
        vecAreas.push_back({pos:{Vector2f(35.79636403f,126.83736494f),Vector2f(35.79623408f,126.83737351f),Vector2f(35.79619695f,126.83625594f),Vector2f(35.79632870f,126.83624870f)}});
        vecAreas.push_back({pos:{Vector2f(35.79735395f,126.83847603f),Vector2f(35.79731730f,126.83736907f),Vector2f(35.79766083f,126.83735188f),Vector2f(35.79769730f,126.83845464f)}});
        vecAreas.push_back({pos:{Vector2f(35.79701050f,126.83849200f),Vector2f(35.79697349f,126.83738759f),Vector2f(35.79731730f,126.83736907f),Vector2f(35.79735395f,126.83847603f)}});
        vecAreas.push_back({pos:{Vector2f(35.79701050f,126.83849200f),Vector2f(35.79665642f,126.83851042f),Vector2f(35.79662202f,126.83740557f),Vector2f(35.79697349f,126.83738759f)}});
        vecAreas.push_back({pos:{Vector2f(35.79641923f,126.83852285f),Vector2f(35.79638276f,126.83741778f),Vector2f(35.79662202f,126.83740557f),Vector2f(35.79665642f,126.83851042f)}});
        vecAreas.push_back({pos:{Vector2f(35.79608531f,126.83851690f),Vector2f(35.79596659f,126.83810473f),Vector2f(35.79578509f,126.83745220f),Vector2f(35.79605437f,126.83743671f)}});
        vecAreas.push_back({pos:{Vector2f(35.79741880f,126.83964754f),Vector2f(35.79737882f,126.83853992f),Vector2f(35.79770073f,126.83852632f),Vector2f(35.79774035f,126.83963063f)}});
        vecAreas.push_back({pos:{Vector2f(35.79741880f,126.83964754f),Vector2f(35.79709860f,126.83966312f),Vector2f(35.79705917f,126.83855495f),Vector2f(35.79737882f,126.83853992f)}});
        vecAreas.push_back({pos:{Vector2f(35.79677661f,126.83968159f),Vector2f(35.79673609f,126.83857276f),Vector2f(35.79705917f,126.83855495f),Vector2f(35.79709860f,126.83966312f)}});
        vecAreas.push_back({pos:{Vector2f(35.79703033f,126.84083722f),Vector2f(35.79699072f,126.83972397f),Vector2f(35.79743815f,126.83969784f),Vector2f(35.79747651f,126.84081099f)}});
        vecAreas.push_back({pos:{Vector2f(35.79703033f,126.84083722f),Vector2f(35.79683089f,126.84084559f),Vector2f(35.79679263f,126.83973244f),Vector2f(35.79699072f,126.83972397f)}});
        vecAreas.push_back({pos:{Vector2f(35.79887098f,126.84090297f),Vector2f(35.79883118f,126.84099218f),Vector2f(35.79881406f,126.84097719f),Vector2f(35.79885109f,126.84089297f)}});
        vecAreas.push_back({pos:{Vector2f(35.79911450f,126.84030161f),Vector2f(35.79912932f,126.84031308f),Vector2f(35.79911039f,126.84035115f),Vector2f(35.79905793f,126.84043093f)}});
        vecAreas.push_back({pos:{Vector2f(35.79996468f,126.84043664f),Vector2f(35.79998883f,126.84039439f),Vector2f(35.80044222f,126.84052432f),Vector2f(35.80046570f,126.84059962f)}});
        vecAreas.push_back({pos:{Vector2f(35.79973372f,126.84195504f),Vector2f(35.79963447f,126.84168499f),Vector2f(35.79984732f,126.84154527f),Vector2f(35.79987793f,126.84189229f)}});
        vecAreas.push_back({pos:{Vector2f(35.79973372f,126.84195504f),Vector2f(35.79959236f,126.84201718f),Vector2f(35.79948313f,126.84178400f),Vector2f(35.79963447f,126.84168499f)}});
        vecAreas.push_back({pos:{Vector2f(35.79913580f,126.84022488f),Vector2f(35.79923456f,126.83999889f),Vector2f(35.79925198f,126.84000953f),Vector2f(35.79923503f,126.84004155f)}});
        vecAreas.push_back({pos:{Vector2f(35.80089827f,126.84203303f),Vector2f(35.80082386f,126.84189150f),Vector2f(35.80093559f,126.84181180f),Vector2f(35.80100778f,126.84196238f)}});
        vecAreas.push_back({pos:{Vector2f(35.80063746f,126.84064851f),Vector2f(35.80046570f,126.84059962f),Vector2f(35.80044222f,126.84052432f),Vector2f(35.80061289f,126.84057303f)}});
        vecAreas.push_back({pos:{Vector2f(35.80057122f,126.84036941f),Vector2f(35.80056459f,126.84031197f),Vector2f(35.80088223f,126.84038165f),Vector2f(35.80090257f,126.84045604f)}});
        vecAreas.push_back({pos:{Vector2f(35.79980623f,126.84040976f),Vector2f(35.79969538f,126.84030520f),Vector2f(35.79983484f,126.84037799f),Vector2f(35.79981356f,126.84041502f)}});
        vecAreas.push_back({pos:{Vector2f(35.79993783f,126.84020127f),Vector2f(35.79995649f,126.84016946f),Vector2f(35.80008311f,126.84020217f),Vector2f(35.80006548f,126.84023590f)}});
        vecAreas.push_back({pos:{Vector2f(35.79981356f,126.84041502f),Vector2f(35.79983484f,126.84037799f),Vector2f(35.79996067f,126.84044367f),Vector2f(35.79993905f,126.84048152f)}});
        vecAreas.push_back({pos:{Vector2f(35.79260248f,126.83129260f),Vector2f(35.79256086f,126.83119972f),Vector2f(35.79261857f,126.83115582f),Vector2f(35.79267083f,126.83124788f)}});
        vecAreas.push_back({pos:{Vector2f(35.79180891f,126.82837848f),Vector2f(35.79200231f,126.82830937f),Vector2f(35.79229774f,126.82950020f),Vector2f(35.79211074f,126.82956985f)}});
        vecAreas.push_back({pos:{Vector2f(35.79201607f,126.82966718f),Vector2f(35.79212740f,126.82962601f),Vector2f(35.79243386f,126.83085035f),Vector2f(35.79232660f,126.83090235f)}});
        vecAreas.push_back({pos:{Vector2f(35.79388361f,126.83295149f),Vector2f(35.79278420f,126.83374375f),Vector2f(35.79270339f,126.83357556f),Vector2f(35.79381352f,126.83277730f)}});
        vecAreas.push_back({pos:{Vector2f(35.79394774f,126.83311076f),Vector2f(35.79285099f,126.83388222f),Vector2f(35.79278420f,126.83374375f),Vector2f(35.79388361f,126.83295149f)}});
        vecAreas.push_back({pos:{Vector2f(35.79403809f,126.83334100f),Vector2f(35.79295575f,126.83410180f),Vector2f(35.79285099f,126.83388222f),Vector2f(35.79394774f,126.83311076f)}});
        vecAreas.push_back({pos:{Vector2f(35.79155402f,126.83370795f),Vector2f(35.79140550f,126.83339400f),Vector2f(35.79226725f,126.83278942f),Vector2f(35.79241540f,126.83310083f)}});
        vecAreas.push_back({pos:{Vector2f(35.79171690f,126.83405075f),Vector2f(35.79155402f,126.83370795f),Vector2f(35.79241540f,126.83310083f),Vector2f(35.79258018f,126.83344540f)}});
        vecAreas.push_back({pos:{Vector2f(35.79188133f,126.83439929f),Vector2f(35.79171690f,126.83405075f),Vector2f(35.79258018f,126.83344540f),Vector2f(35.79274487f,126.83378875f)}});
        vecAreas.push_back({pos:{Vector2f(35.79204548f,126.83474552f),Vector2f(35.79188133f,126.83439929f),Vector2f(35.79274487f,126.83378875f),Vector2f(35.79290911f,126.83413398f)}});
        vecAreas.push_back({pos:{Vector2f(35.79220203f,126.83507450f),Vector2f(35.79204548f,126.83474552f),Vector2f(35.79290911f,126.83413398f),Vector2f(35.79305781f,126.83444860f)}});
        vecAreas.push_back({pos:{Vector2f(35.79984570f,126.82138652f),Vector2f(35.79953681f,126.82154851f),Vector2f(35.79949309f,126.82027892f),Vector2f(35.79980526f,126.82026316f)}});
        vecAreas.push_back({pos:{Vector2f(35.79921964f,126.82147632f),Vector2f(35.79886476f,126.82149659f),Vector2f(35.79882423f,126.82031296f),Vector2f(35.79917992f,126.82029423f)}});
        vecAreas.push_back({pos:{Vector2f(35.79859361f,126.82206406f),Vector2f(35.79828993f,126.82221829f),Vector2f(35.79826709f,126.82158932f),Vector2f(35.79857440f,126.82157391f)}});
        vecAreas.push_back({pos:{Vector2f(35.79828993f,126.82221829f),Vector2f(35.79797537f,126.82238417f),Vector2f(35.79794636f,126.82160586f),Vector2f(35.79826709f,126.82158932f)}});
        vecAreas.push_back({pos:{Vector2f(35.79797537f,126.82238417f),Vector2f(35.79765693f,126.82254938f),Vector2f(35.79762373f,126.82162031f),Vector2f(35.79794636f,126.82160586f)}});
        vecAreas.push_back({pos:{Vector2f(35.79765693f,126.82254938f),Vector2f(35.79735138f,126.82271202f),Vector2f(35.79731246f,126.82163683f),Vector2f(35.79762373f,126.82162031f)}});
        vecAreas.push_back({pos:{Vector2f(35.79485433f,126.82898904f),Vector2f(35.79491999f,126.82902585f),Vector2f(35.79507198f,126.82917652f),Vector2f(35.79486173f,126.82918726f)}});
        vecAreas.push_back({pos:{Vector2f(35.79709565f,126.82973535f),Vector2f(35.79690321f,126.82959504f),Vector2f(35.79688789f,126.82915213f),Vector2f(35.79707777f,126.82914298f)}});
        vecAreas.push_back({pos:{Vector2f(35.79638970f,126.83036898f),Vector2f(35.79592198f,126.83039055f),Vector2f(35.79590114f,126.82981226f),Vector2f(35.79638293f,126.83017329f)}});
        vecAreas.push_back({pos:{Vector2f(35.79490426f,126.83044484f),Vector2f(35.79456902f,126.83046414f),Vector2f(35.79453037f,126.82927172f),Vector2f(35.79486354f,126.82925651f)}});
        vecAreas.push_back({pos:{Vector2f(35.79456902f,126.83046414f),Vector2f(35.79423874f,126.83048144f),Vector2f(35.79419558f,126.82928815f),Vector2f(35.79453037f,126.82927172f)}});
        vecAreas.push_back({pos:{Vector2f(35.79356230f,126.83051707f),Vector2f(35.79322526f,126.83053693f),Vector2f(35.79318597f,126.82933911f),Vector2f(35.79352194f,126.82932389f)}});
        vecAreas.push_back({pos:{Vector2f(35.79703842f,126.83150747f),Vector2f(35.79701029f,126.83067684f),Vector2f(35.79732155f,126.83090672f),Vector2f(35.79734257f,126.83149278f)}});
        vecAreas.push_back({pos:{Vector2f(35.79670452f,126.83152389f),Vector2f(35.79636613f,126.83154165f),Vector2f(35.79632670f,126.83043283f),Vector2f(35.79666618f,126.83041628f)}});
        vecAreas.push_back({pos:{Vector2f(35.79620544f,126.83155007f),Vector2f(35.79595428f,126.83156321f),Vector2f(35.79591360f,126.83045407f),Vector2f(35.79616692f,126.83044070f)}});
        vecAreas.push_back({pos:{Vector2f(35.79595428f,126.83156321f),Vector2f(35.79566248f,126.83157855f),Vector2f(35.79562503f,126.83046940f),Vector2f(35.79591360f,126.83045407f)}});
        vecAreas.push_back({pos:{Vector2f(35.79566248f,126.83157855f),Vector2f(35.79528902f,126.83159715f),Vector2f(35.79524698f,126.83048802f),Vector2f(35.79562503f,126.83046940f)}});
        vecAreas.push_back({pos:{Vector2f(35.79528902f,126.83159715f),Vector2f(35.79494252f,126.83161625f),Vector2f(35.79490480f,126.83050491f),Vector2f(35.79524698f,126.83048802f)}});
        vecAreas.push_back({pos:{Vector2f(35.79494252f,126.83161625f),Vector2f(35.79461484f,126.83163421f),Vector2f(35.79457560f,126.83052397f),Vector2f(35.79490480f,126.83050491f)}});
        vecAreas.push_back({pos:{Vector2f(35.79461484f,126.83163421f),Vector2f(35.79428474f,126.83165073f),Vector2f(35.79424477f,126.83054260f),Vector2f(35.79457560f,126.83052397f)}});
        vecAreas.push_back({pos:{Vector2f(35.79389784f,126.83249393f),Vector2f(35.79386776f,126.83250561f),Vector2f(35.79357183f,126.83176795f),Vector2f(35.79387055f,126.83173480f)}});
        vecAreas.push_back({pos:{Vector2f(35.79578035f,126.83362218f),Vector2f(35.79568770f,126.83362592f),Vector2f(35.79565962f,126.83281378f),Vector2f(35.79575163f,126.83280905f)}});
        vecAreas.push_back({pos:{Vector2f(35.79568770f,126.83362592f),Vector2f(35.79519763f,126.83364863f),Vector2f(35.79517469f,126.83283881f),Vector2f(35.79565962f,126.83281378f)}});
        vecAreas.push_back({pos:{Vector2f(35.79671621f,126.82285638f),Vector2f(35.79638916f,126.82287237f),Vector2f(35.79634810f,126.82168801f),Vector2f(35.79667866f,126.82166923f)}});
        vecAreas.push_back({pos:{Vector2f(35.79638916f,126.82287237f),Vector2f(35.79606275f,126.82288926f),Vector2f(35.79601845f,126.82170302f),Vector2f(35.79634810f,126.82168801f)}});
        vecAreas.push_back({pos:{Vector2f(35.79606275f,126.82288926f),Vector2f(35.79573563f,126.82290868f),Vector2f(35.79569465f,126.82172023f),Vector2f(35.79601845f,126.82170302f)}});
        vecAreas.push_back({pos:{Vector2f(35.79650981f,126.82535785f),Vector2f(35.79637265f,126.82536623f),Vector2f(35.79633304f,126.82416758f),Vector2f(35.79646669f,126.82416120f)}});
        vecAreas.push_back({pos:{Vector2f(35.79637265f,126.82536623f),Vector2f(35.79606715f,126.82538537f),Vector2f(35.79602646f,126.82418497f),Vector2f(35.79633304f,126.82416758f)}});
        vecAreas.push_back({pos:{Vector2f(35.79606715f,126.82538537f),Vector2f(35.79574750f,126.82540035f),Vector2f(35.79570581f,126.82420128f),Vector2f(35.79602646f,126.82418497f)}});
        vecAreas.push_back({pos:{Vector2f(35.79574750f,126.82540035f),Vector2f(35.79540802f,126.82541703f),Vector2f(35.79536832f,126.82421763f),Vector2f(35.79570581f,126.82420128f)}});
        vecAreas.push_back({pos:{Vector2f(35.79540802f,126.82541703f),Vector2f(35.79506953f,126.82543437f),Vector2f(35.79502830f,126.82423641f),Vector2f(35.79536832f,126.82421763f)}});
        vecAreas.push_back({pos:{Vector2f(35.79506953f,126.82543437f),Vector2f(35.79473024f,126.82545259f),Vector2f(35.79469000f,126.82425542f),Vector2f(35.79502830f,126.82423641f)}});
        vecAreas.push_back({pos:{Vector2f(35.79473024f,126.82545259f),Vector2f(35.79439707f,126.82547103f),Vector2f(35.79435674f,126.82427231f),Vector2f(35.79469000f,126.82425542f)}});
        vecAreas.push_back({pos:{Vector2f(35.79375001f,126.82550111f),Vector2f(35.79343388f,126.82552006f),Vector2f(35.79339084f,126.82432202f),Vector2f(35.79370554f,126.82430849f)}});
        vecAreas.push_back({pos:{Vector2f(35.79685083f,126.82658161f),Vector2f(35.79680938f,126.82541604f),Vector2f(35.79712453f,126.82539798f),Vector2f(35.79717030f,126.82656498f)}});
        vecAreas.push_back({pos:{Vector2f(35.79654344f,126.82659877f),Vector2f(35.79604148f,126.82662564f),Vector2f(35.79600129f,126.82546019f),Vector2f(35.79650352f,126.82543154f)}});
        vecAreas.push_back({pos:{Vector2f(35.79604148f,126.82662564f),Vector2f(35.79579094f,126.82663769f),Vector2f(35.79575022f,126.82547192f),Vector2f(35.79600129f,126.82546019f)}});
        vecAreas.push_back({pos:{Vector2f(35.79579094f,126.82663769f),Vector2f(35.79545165f,126.82665481f),Vector2f(35.79541065f,126.82548937f),Vector2f(35.79575022f,126.82547192f)}});
        vecAreas.push_back({pos:{Vector2f(35.79545165f,126.82665481f),Vector2f(35.79511199f,126.82667480f),Vector2f(35.79507208f,126.82550782f),Vector2f(35.79541065f,126.82548937f)}});
        vecAreas.push_back({pos:{Vector2f(35.79511199f,126.82667480f),Vector2f(35.79477413f,126.82669213f),Vector2f(35.79473305f,126.82552505f),Vector2f(35.79507208f,126.82550782f)}});
        vecAreas.push_back({pos:{Vector2f(35.79477413f,126.82669213f),Vector2f(35.79443970f,126.82670858f),Vector2f(35.79439988f,126.82554138f),Vector2f(35.79473305f,126.82552505f)}});
        vecAreas.push_back({pos:{Vector2f(35.79443970f,126.82670858f),Vector2f(35.79410824f,126.82672589f),Vector2f(35.79406716f,126.82556048f),Vector2f(35.79439988f,126.82554138f)}});
        vecAreas.push_back({pos:{Vector2f(35.79379193f,126.82674429f),Vector2f(35.79347759f,126.82675991f),Vector2f(35.79343795f,126.82559184f),Vector2f(35.79375102f,126.82557411f)}});
        vecAreas.push_back({pos:{Vector2f(35.79704072f,126.82782186f),Vector2f(35.79700269f,126.82664046f),Vector2f(35.79717086f,126.82663511f),Vector2f(35.79720952f,126.82781618f)}});
        vecAreas.push_back({pos:{Vector2f(35.79704072f,126.82782186f),Vector2f(35.79683768f,126.82783370f),Vector2f(35.79679731f,126.82665341f),Vector2f(35.79700269f,126.82664046f)}});
        vecAreas.push_back({pos:{Vector2f(35.79658030f,126.82784344f),Vector2f(35.79648099f,126.82784952f),Vector2f(35.79643630f,126.82667312f),Vector2f(35.79653813f,126.82666604f)}});
        vecAreas.push_back({pos:{Vector2f(35.79648099f,126.82784952f),Vector2f(35.79610393f,126.82786704f),Vector2f(35.79606221f,126.82669064f),Vector2f(35.79643630f,126.82667312f)}});
        vecAreas.push_back({pos:{Vector2f(35.79610393f,126.82786704f),Vector2f(35.79583474f,126.82788146f),Vector2f(35.79579077f,126.82670540f),Vector2f(35.79606221f,126.82669064f)}});
        vecAreas.push_back({pos:{Vector2f(35.79583474f,126.82788146f),Vector2f(35.79549500f,126.82789934f),Vector2f(35.79545328f,126.82672195f),Vector2f(35.79579077f,126.82670540f)}});
        vecAreas.push_back({pos:{Vector2f(35.79549500f,126.82789934f),Vector2f(35.79515561f,126.82791723f),Vector2f(35.79511218f,126.82674095f),Vector2f(35.79545328f,126.82672195f)}});
        vecAreas.push_back({pos:{Vector2f(35.79515561f,126.82791723f),Vector2f(35.79481685f,126.82793356f),Vector2f(35.79477648f,126.82675408f),Vector2f(35.79511218f,126.82674095f)}});
        vecAreas.push_back({pos:{Vector2f(35.79447412f,126.82769514f),Vector2f(35.79412931f,126.82739422f),Vector2f(35.79410852f,126.82679083f),Vector2f(35.79444214f,126.82677362f)}});
        vecAreas.push_back({pos:{Vector2f(35.79380290f,126.82710024f),Vector2f(35.79363911f,126.82695270f),Vector2f(35.79348879f,126.82682449f),Vector2f(35.79379265f,126.82680822f)}});
        vecAreas.push_back({pos:{Vector2f(35.79452694f,126.82920624f),Vector2f(35.79435626f,126.82921435f),Vector2f(35.79433300f,126.82852378f),Vector2f(35.79450842f,126.82867153f)}});
        vecAreas.push_back({pos:{Vector2f(35.80850291f,126.82566297f),Vector2f(35.80830906f,126.82679161f),Vector2f(35.80798905f,126.82670801f),Vector2f(35.80818221f,126.82557804f)}});
        vecAreas.push_back({pos:{Vector2f(35.80713164f,126.83040502f),Vector2f(35.80735056f,126.83046141f),Vector2f(35.80772447f,126.83055772f),Vector2f(35.80738548f,126.83086229f)}});
        vecAreas.push_back({pos:{Vector2f(35.80790031f,126.83180440f),Vector2f(35.80814064f,126.83228730f),Vector2f(35.80809635f,126.83232584f),Vector2f(35.80785350f,126.83184614f)}});
        vecAreas.push_back({pos:{Vector2f(35.80584986f,126.82361947f),Vector2f(35.80604397f,126.82245402f),Vector2f(35.80636161f,126.82253750f),Vector2f(35.80616668f,126.82370196f)}});
        vecAreas.push_back({pos:{Vector2f(35.80666189f,126.82514990f),Vector2f(35.80686753f,126.82396892f),Vector2f(35.80719184f,126.82405528f),Vector2f(35.80698656f,126.82523637f)}});
        vecAreas.push_back({pos:{Vector2f(35.80666189f,126.82514990f),Vector2f(35.80633884f,126.82506476f),Vector2f(35.80654241f,126.82388268f),Vector2f(35.80686753f,126.82396892f)}});
        vecAreas.push_back({pos:{Vector2f(35.80633884f,126.82506476f),Vector2f(35.80601445f,126.82498128f),Vector2f(35.80621676f,126.82380065f),Vector2f(35.80654241f,126.82388268f)}});
        vecAreas.push_back({pos:{Vector2f(35.80601445f,126.82498128f),Vector2f(35.80567608f,126.82489241f),Vector2f(35.80586586f,126.82371204f),Vector2f(35.80621676f,126.82380065f)}});
        vecAreas.push_back({pos:{Vector2f(35.80786206f,126.82549477f),Vector2f(35.80766507f,126.82662318f),Vector2f(35.80734775f,126.82653980f),Vector2f(35.80754469f,126.82540917f)}});
        vecAreas.push_back({pos:{Vector2f(35.80840147f,126.82816922f),Vector2f(35.80860679f,126.82699586f),Vector2f(35.80892767f,126.82707969f),Vector2f(35.80872649f,126.82825283f)}});
        vecAreas.push_back({pos:{Vector2f(35.80808474f,126.82808671f),Vector2f(35.80828978f,126.82690993f),Vector2f(35.80860679f,126.82699586f),Vector2f(35.80840147f,126.82816922f)}});
        vecAreas.push_back({pos:{Vector2f(35.80776512f,126.82800343f),Vector2f(35.80744000f,126.82791761f),Vector2f(35.80764325f,126.82674261f),Vector2f(35.80797089f,126.82682842f)}});
        vecAreas.push_back({pos:{Vector2f(35.80679942f,126.82775216f),Vector2f(35.80700140f,126.82657551f),Vector2f(35.80732616f,126.82666044f),Vector2f(35.80712508f,126.82783709f)}});
        vecAreas.push_back({pos:{Vector2f(35.80647945f,126.82766922f),Vector2f(35.80668061f,126.82649158f),Vector2f(35.80700140f,126.82657551f),Vector2f(35.80679942f,126.82775216f)}});
        vecAreas.push_back({pos:{Vector2f(35.80647945f,126.82766922f),Vector2f(35.80615749f,126.82758539f),Vector2f(35.80636018f,126.82640543f),Vector2f(35.80668061f,126.82649158f)}});
        vecAreas.push_back({pos:{Vector2f(35.80615749f,126.82758539f),Vector2f(35.80584319f,126.82750454f),Vector2f(35.80604715f,126.82632557f),Vector2f(35.80636018f,126.82640543f)}});
        vecAreas.push_back({pos:{Vector2f(35.80552159f,126.82741817f),Vector2f(35.80572519f,126.82624153f),Vector2f(35.80604715f,126.82632557f),Vector2f(35.80584319f,126.82750454f)}});
        vecAreas.push_back({pos:{Vector2f(35.80883359f,126.82953195f),Vector2f(35.80851328f,126.82946504f),Vector2f(35.80871779f,126.82828515f),Vector2f(35.80903696f,126.82836866f)}});
        vecAreas.push_back({pos:{Vector2f(35.80851328f,126.82946504f),Vector2f(35.80819529f,126.82937998f),Vector2f(35.80839683f,126.82820066f),Vector2f(35.80871779f,126.82828515f)}});
        vecAreas.push_back({pos:{Vector2f(35.80787865f,126.82929679f),Vector2f(35.80807919f,126.82811615f),Vector2f(35.80839683f,126.82820066f),Vector2f(35.80819529f,126.82937998f)}});
        vecAreas.push_back({pos:{Vector2f(35.80787865f,126.82929679f),Vector2f(35.80771771f,126.82925377f),Vector2f(35.80792051f,126.82807523f),Vector2f(35.80807919f,126.82811615f)}});
        vecAreas.push_back({pos:{Vector2f(35.80771771f,126.82925377f),Vector2f(35.80755668f,126.82921064f),Vector2f(35.80776183f,126.82803441f),Vector2f(35.80792051f,126.82807523f)}});
        vecAreas.push_back({pos:{Vector2f(35.80723616f,126.82912470f),Vector2f(35.80743690f,126.82794915f),Vector2f(35.80776183f,126.82803441f),Vector2f(35.80755668f,126.82921064f)}});
        vecAreas.push_back({pos:{Vector2f(35.80659738f,126.82895226f),Vector2f(35.80679658f,126.82778293f),Vector2f(35.80711368f,126.82786588f),Vector2f(35.80691510f,126.82903644f)}});
        vecAreas.push_back({pos:{Vector2f(35.80881713f,126.82961840f),Vector2f(35.80881211f,126.82963733f),Vector2f(35.80803485f,126.83031734f),Vector2f(35.80818303f,126.82944351f)}});
        vecAreas.push_back({pos:{Vector2f(35.80704787f,126.83024610f),Vector2f(35.80633928f,126.82896643f),Vector2f(35.80634287f,126.82895724f),Vector2f(35.80722779f,126.82919453f)}});
        vecAreas.push_back({pos:{Vector2f(35.80882147f,126.83016413f),Vector2f(35.80939178f,126.83115522f),Vector2f(35.80905825f,126.83144405f),Vector2f(35.80848624f,126.83045671f)}});
        vecAreas.push_back({pos:{Vector2f(35.80905825f,126.83144405f),Vector2f(35.80876732f,126.83169869f),Vector2f(35.80819241f,126.83070732f),Vector2f(35.80848624f,126.83045671f)}});
        vecAreas.push_back({pos:{Vector2f(35.80507580f,126.83959624f),Vector2f(35.80480853f,126.83983045f),Vector2f(35.80423617f,126.83884551f),Vector2f(35.80450353f,126.83861319f)}});
        vecAreas.push_back({pos:{Vector2f(35.80348997f,126.83941293f),Vector2f(35.80339815f,126.83949300f),Vector2f(35.80282153f,126.83849271f),Vector2f(35.80291299f,126.83841165f)}});
        vecAreas.push_back({pos:{Vector2f(35.80339815f,126.83949300f),Vector2f(35.80313079f,126.83972576f),Vector2f(35.80255282f,126.83872835f),Vector2f(35.80282153f,126.83849271f)}});
        vecAreas.push_back({pos:{Vector2f(35.80280655f,126.84001417f),Vector2f(35.80261237f,126.84017974f),Vector2f(35.80204124f,126.83917403f),Vector2f(35.80223082f,126.83901079f)}});
        vecAreas.push_back({pos:{Vector2f(35.80911239f,126.83300575f),Vector2f(35.80968831f,126.83399408f),Vector2f(35.80941719f,126.83422897f),Vector2f(35.80884203f,126.83323770f)}});
        vecAreas.push_back({pos:{Vector2f(35.80883036f,126.83629514f),Vector2f(35.80855904f,126.83653192f),Vector2f(35.80799225f,126.83555348f),Vector2f(35.80826041f,126.83531759f)}});
        vecAreas.push_back({pos:{Vector2f(35.80855904f,126.83653192f),Vector2f(35.80829628f,126.83676282f),Vector2f(35.80772463f,126.83578594f),Vector2f(35.80799225f,126.83555348f)}});
        vecAreas.push_back({pos:{Vector2f(35.80829628f,126.83676282f),Vector2f(35.80815937f,126.83688249f),Vector2f(35.80758942f,126.83590416f),Vector2f(35.80772463f,126.83578594f)}});
        vecAreas.push_back({pos:{Vector2f(35.80815937f,126.83688249f),Vector2f(35.80802308f,126.83700171f),Vector2f(35.80745529f,126.83602128f),Vector2f(35.80758942f,126.83590416f)}});
        vecAreas.push_back({pos:{Vector2f(35.80802308f,126.83700171f),Vector2f(35.80789039f,126.83711749f),Vector2f(35.80732360f,126.83613872f),Vector2f(35.80745529f,126.83602128f)}});
        vecAreas.push_back({pos:{Vector2f(35.80789039f,126.83711749f),Vector2f(35.80775861f,126.83723250f),Vector2f(35.80719047f,126.83625738f),Vector2f(35.80732360f,126.83613872f)}});
        vecAreas.push_back({pos:{Vector2f(35.80775861f,126.83723250f),Vector2f(35.80749404f,126.83746350f),Vector2f(35.80692122f,126.83648718f),Vector2f(35.80719047f,126.83625738f)}});
        vecAreas.push_back({pos:{Vector2f(35.80749404f,126.83746350f),Vector2f(35.80729033f,126.83764338f),Vector2f(35.80671959f,126.83667159f),Vector2f(35.80692122f,126.83648718f)}});
        vecAreas.push_back({pos:{Vector2f(35.80729033f,126.83764338f),Vector2f(35.80723011f,126.83769639f),Vector2f(35.80665910f,126.83672405f),Vector2f(35.80671959f,126.83667159f)}});
        vecAreas.push_back({pos:{Vector2f(35.80723011f,126.83769639f),Vector2f(35.80696285f,126.83793194f),Vector2f(35.80639039f,126.83695728f),Vector2f(35.80665910f,126.83672405f)}});
        vecAreas.push_back({pos:{Vector2f(35.80696285f,126.83793194f),Vector2f(35.80688273f,126.83800258f),Vector2f(35.80630983f,126.83702715f),Vector2f(35.80639039f,126.83695728f)}});
        vecAreas.push_back({pos:{Vector2f(35.80640366f,126.83842310f),Vector2f(35.80613819f,126.83865565f),Vector2f(35.80556916f,126.83767413f),Vector2f(35.80583201f,126.83744335f)}});
        vecAreas.push_back({pos:{Vector2f(35.80613819f,126.83865565f),Vector2f(35.80595698f,126.83881390f),Vector2f(35.80538714f,126.83783073f),Vector2f(35.80556916f,126.83767413f)}});
        vecAreas.push_back({pos:{Vector2f(35.80595698f,126.83881390f),Vector2f(35.80574463f,126.83900242f),Vector2f(35.80516957f,126.83802556f),Vector2f(35.80538714f,126.83783073f)}});
        vecAreas.push_back({pos:{Vector2f(35.80574463f,126.83900242f),Vector2f(35.80560807f,126.83912296f),Vector2f(35.80503472f,126.83814267f),Vector2f(35.80516957f,126.83802556f)}});
        vecAreas.push_back({pos:{Vector2f(35.80560807f,126.83912296f),Vector2f(35.80534819f,126.83935605f),Vector2f(35.80477150f,126.83837234f),Vector2f(35.80503472f,126.83814267f)}});
        vecAreas.push_back({pos:{Vector2f(35.80534819f,126.83935605f),Vector2f(35.80507580f,126.83959624f),Vector2f(35.80450353f,126.83861319f),Vector2f(35.80477150f,126.83837234f)}});
        vecAreas.push_back({pos:{Vector2f(35.80832292f,126.83208365f),Vector2f(35.80774800f,126.83109235f),Vector2f(35.80793063f,126.83093446f),Vector2f(35.80850373f,126.83192506f)}});
        vecAreas.push_back({pos:{Vector2f(35.80753397f,126.83121275f),Vector2f(35.80762991f,126.83132291f),Vector2f(35.80759383f,126.83135454f),Vector2f(35.80752385f,126.83121661f)}});
        vecAreas.push_back({pos:{Vector2f(35.79831829f,126.82611121f),Vector2f(35.79739701f,126.82616125f),Vector2f(35.79738423f,126.82574355f),Vector2f(35.79830200f,126.82569120f)}});
        vecAreas.push_back({pos:{Vector2f(35.79749541f,126.82908079f),Vector2f(35.79748599f,126.82866630f),Vector2f(35.79840303f,126.82861342f),Vector2f(35.79841463f,126.82903323f)}});
        vecAreas.push_back({pos:{Vector2f(35.79768906f,126.83455418f),Vector2f(35.79767897f,126.83435640f),Vector2f(35.79861206f,126.83430765f),Vector2f(35.79861829f,126.83450965f)}});
        vecAreas.push_back({pos:{Vector2f(35.79782990f,126.83880804f),Vector2f(35.79781422f,126.83841115f),Vector2f(35.79872507f,126.83836846f),Vector2f(35.79873868f,126.83876747f)}});
        vecAreas.push_back({pos:{Vector2f(35.80391741f,126.84039538f),Vector2f(35.80402167f,126.84040542f),Vector2f(35.80412949f,126.84042563f),Vector2f(35.80399937f,126.84053700f)}});
        vecAreas.push_back({pos:{Vector2f(35.79674057f,126.83499900f),Vector2f(35.79640974f,126.83501684f),Vector2f(35.79637079f,126.83390326f),Vector2f(35.79670288f,126.83388320f)}});
        vecAreas.push_back({pos:{Vector2f(35.79728649f,126.83613454f),Vector2f(35.79699658f,126.83614986f),Vector2f(35.79695810f,126.83503881f),Vector2f(35.79724783f,126.83502460f)}});
        vecAreas.push_back({pos:{Vector2f(35.79286838f,126.83734481f),Vector2f(35.79260959f,126.83711846f),Vector2f(35.79314406f,126.83621315f),Vector2f(35.79340412f,126.83644613f)}});
        vecAreas.push_back({pos:{Vector2f(35.80698656f,126.82523637f),Vector2f(35.80719184f,126.82405528f),Vector2f(35.80751867f,126.82414085f),Vector2f(35.80731249f,126.82532172f)}});
        vecAreas.push_back({pos:{Vector2f(35.80712508f,126.82783709f),Vector2f(35.80732616f,126.82666044f),Vector2f(35.80764325f,126.82674261f),Vector2f(35.80744000f,126.82791761f)}});
        vecAreas.push_back({pos:{Vector2f(35.80723616f,126.82912470f),Vector2f(35.80691510f,126.82903644f),Vector2f(35.80711368f,126.82786588f),Vector2f(35.80743690f,126.82794915f)}});
        vecAreas.push_back({pos:{Vector2f(35.80734427f,126.83042006f),Vector2f(35.80754290f,126.82927639f),Vector2f(35.80786774f,126.82936110f),Vector2f(35.80766335f,126.83050335f)}});
        vecAreas.push_back({pos:{Vector2f(35.79592198f,126.83039055f),Vector2f(35.79558385f,126.83041086f),Vector2f(35.79555333f,126.82954618f),Vector2f(35.79590114f,126.82981226f)}});
        vecAreas.push_back({pos:{Vector2f(35.79390025f,126.83049920f),Vector2f(35.79356230f,126.83051707f),Vector2f(35.79352194f,126.82932389f),Vector2f(35.79386097f,126.82930635f)}});
        vecAreas.push_back({pos:{Vector2f(35.79636613f,126.83154165f),Vector2f(35.79620544f,126.83155007f),Vector2f(35.79616692f,126.83044070f),Vector2f(35.79632670f,126.83043283f)}});
        vecAreas.push_back({pos:{Vector2f(35.80229330f,126.83633302f),Vector2f(35.80220792f,126.83643605f),Vector2f(35.80206158f,126.83624022f),Vector2f(35.80216628f,126.83613410f)}});
        vecAreas.push_back({pos:{Vector2f(35.80603530f,126.83009897f),Vector2f(35.80590830f,126.82986374f),Vector2f(35.80590569f,126.82978661f),Vector2f(35.80601019f,126.83000912f)}});
        vecAreas.push_back({pos:{Vector2f(35.79696169f,126.83356209f),Vector2f(35.79651176f,126.83358480f),Vector2f(35.79649271f,126.83277126f),Vector2f(35.79693824f,126.83274989f)}});
        vecAreas.push_back({pos:{Vector2f(35.80963076f,126.83094708f),Vector2f(35.80939178f,126.83115522f),Vector2f(35.80882147f,126.83016413f),Vector2f(35.80905934f,126.82995949f)}});
        vecAreas.push_back({pos:{Vector2f(35.80929809f,126.82975359f),Vector2f(35.80986937f,126.83073798f),Vector2f(35.80963076f,126.83094708f),Vector2f(35.80905934f,126.82995949f)}});
        vecAreas.push_back({pos:{Vector2f(35.80819241f,126.83070732f),Vector2f(35.80876732f,126.83169869f),Vector2f(35.80850373f,126.83192506f),Vector2f(35.80793063f,126.83093446f)}});
        vecAreas.push_back({pos:{Vector2f(35.80874088f,126.83324101f),Vector2f(35.80817245f,126.83226527f),Vector2f(35.80851191f,126.83196178f),Vector2f(35.80908047f,126.83294512f)}});
        vecAreas.push_back({pos:{Vector2f(35.80877725f,126.83172855f),Vector2f(35.80934678f,126.83271227f),Vector2f(35.80908047f,126.83294512f),Vector2f(35.80851191f,126.83196178f)}});
        vecAreas.push_back({pos:{Vector2f(35.80961672f,126.83247785f),Vector2f(35.80934678f,126.83271227f),Vector2f(35.80877725f,126.83172855f),Vector2f(35.80904794f,126.83149323f)}});
        vecAreas.push_back({pos:{Vector2f(35.80930943f,126.83126672f),Vector2f(35.80988109f,126.83225015f),Vector2f(35.80961672f,126.83247785f),Vector2f(35.80904794f,126.83149323f)}});
        vecAreas.push_back({pos:{Vector2f(35.80227662f,126.84032918f),Vector2f(35.80233349f,126.84032099f),Vector2f(35.80246725f,126.84031779f),Vector2f(35.80233556f,126.84043376f)}});
        vecAreas.push_back({pos:{Vector2f(35.79687004f,126.84188782f),Vector2f(35.79684841f,126.84132234f),Vector2f(35.79726530f,126.84121906f),Vector2f(35.79728614f,126.84180557f)}});
        vecAreas.push_back({pos:{Vector2f(35.79683269f,126.84091163f),Vector2f(35.79725355f,126.84088844f),Vector2f(35.79726530f,126.84121906f),Vector2f(35.79684841f,126.84132234f)}});
        vecAreas.push_back({pos:{Vector2f(35.80157168f,126.83013538f),Vector2f(35.80061020f,126.83018314f),Vector2f(35.80059357f,126.82977041f),Vector2f(35.80155731f,126.82972475f)}});
        vecAreas.push_back({pos:{Vector2f(35.80158522f,126.83053098f),Vector2f(35.80062176f,126.83057829f),Vector2f(35.80061020f,126.83018314f),Vector2f(35.80157168f,126.83013538f)}});
        vecAreas.push_back({pos:{Vector2f(35.80159598f,126.83093266f),Vector2f(35.80063522f,126.83098239f),Vector2f(35.80062176f,126.83057829f),Vector2f(35.80158522f,126.83053098f)}});
        vecAreas.push_back({pos:{Vector2f(35.80160854f,126.83133788f),Vector2f(35.80064787f,126.83139015f),Vector2f(35.80063522f,126.83098239f),Vector2f(35.80159598f,126.83093266f)}});
        vecAreas.push_back({pos:{Vector2f(35.80162073f,126.83173878f),Vector2f(35.80066204f,126.83178795f),Vector2f(35.80064787f,126.83139015f),Vector2f(35.80160854f,126.83133788f)}});
        vecAreas.push_back({pos:{Vector2f(35.80226853f,126.82119729f),Vector2f(35.80131670f,126.82124701f),Vector2f(35.80130331f,126.82084799f),Vector2f(35.80225397f,126.82080258f)}});
        vecAreas.push_back({pos:{Vector2f(35.80228366f,126.82160892f),Vector2f(35.80133020f,126.82165266f),Vector2f(35.80131670f,126.82124701f),Vector2f(35.80226853f,126.82119729f)}});
        vecAreas.push_back({pos:{Vector2f(35.80229560f,126.82199954f),Vector2f(35.80134440f,126.82204759f),Vector2f(35.80133020f,126.82165266f),Vector2f(35.80228366f,126.82160892f)}});
        vecAreas.push_back({pos:{Vector2f(35.80230855f,126.82240564f),Vector2f(35.80135815f,126.82244804f),Vector2f(35.80134440f,126.82204759f),Vector2f(35.80229560f,126.82199954f)}});
        vecAreas.push_back({pos:{Vector2f(35.80236424f,126.82440764f),Vector2f(35.80142430f,126.82445708f),Vector2f(35.80141172f,126.82405164f),Vector2f(35.80235202f,126.82400165f)}});
        vecAreas.push_back({pos:{Vector2f(35.80238124f,126.82480998f),Vector2f(35.80143823f,126.82485886f),Vector2f(35.80142430f,126.82445708f),Vector2f(35.80236424f,126.82440764f)}});
        vecAreas.push_back({pos:{Vector2f(35.80239878f,126.82521652f),Vector2f(35.80145045f,126.82526386f),Vector2f(35.80143823f,126.82485886f),Vector2f(35.80238124f,126.82480998f)}});
        vecAreas.push_back({pos:{Vector2f(35.80241179f,126.82561333f),Vector2f(35.80146437f,126.82566188f),Vector2f(35.80145045f,126.82526386f),Vector2f(35.80239878f,126.82521652f)}});
        vecAreas.push_back({pos:{Vector2f(35.80246191f,126.82685430f),Vector2f(35.80150756f,126.82690728f),Vector2f(35.80149409f,126.82650981f),Vector2f(35.80244764f,126.82646137f)}});
        vecAreas.push_back({pos:{Vector2f(35.80247422f,126.82726240f),Vector2f(35.80151950f,126.82731117f),Vector2f(35.80150756f,126.82690728f),Vector2f(35.80246191f,126.82685430f)}});
        vecAreas.push_back({pos:{Vector2f(35.80249094f,126.82767182f),Vector2f(35.80153424f,126.82771738f),Vector2f(35.80151950f,126.82731117f),Vector2f(35.80247422f,126.82726240f)}});
        vecAreas.push_back({pos:{Vector2f(35.80175231f,126.83535197f),Vector2f(35.80078921f,126.83540332f),Vector2f(35.80077496f,126.83500397f),Vector2f(35.80173851f,126.83495262f)}});
        vecAreas.push_back({pos:{Vector2f(35.80173851f,126.83495262f),Vector2f(35.80077496f,126.83500397f),Vector2f(35.80076114f,126.83459223f),Vector2f(35.80172470f,126.83454762f)}});
        vecAreas.push_back({pos:{Vector2f(35.80172470f,126.83454762f),Vector2f(35.80076114f,126.83459223f),Vector2f(35.80074671f,126.83419731f),Vector2f(35.80170955f,126.83415159f)}});
        vecAreas.push_back({pos:{Vector2f(35.80170955f,126.83415159f),Vector2f(35.80074671f,126.83419731f),Vector2f(35.80074103f,126.83399752f),Vector2f(35.80170332f,126.83394616f)}});
        vecAreas.push_back({pos:{Vector2f(35.80170332f,126.83394616f),Vector2f(35.80074103f,126.83399752f),Vector2f(35.80073363f,126.83379752f),Vector2f(35.80169682f,126.83374670f)}});
        vecAreas.push_back({pos:{Vector2f(35.80169682f,126.83374670f),Vector2f(35.80073363f,126.83379752f),Vector2f(35.80072019f,126.83340171f),Vector2f(35.80168284f,126.83334790f)}});
        vecAreas.push_back({pos:{Vector2f(35.80272586f,126.83490265f),Vector2f(35.80177699f,126.83494446f),Vector2f(35.80176184f,126.83454777f),Vector2f(35.80271521f,126.83450340f)}});
        vecAreas.push_back({pos:{Vector2f(35.80271521f,126.83450340f),Vector2f(35.80176184f,126.83454777f),Vector2f(35.80174722f,126.83414764f),Vector2f(35.80269960f,126.83410161f)}});
        vecAreas.push_back({pos:{Vector2f(35.80269960f,126.83410161f),Vector2f(35.80174722f,126.83414764f),Vector2f(35.80173982f,126.83394819f),Vector2f(35.80269346f,126.83389751f)}});
        vecAreas.push_back({pos:{Vector2f(35.79946658f,126.82979463f),Vector2f(35.79850294f,126.82984140f),Vector2f(35.79848795f,126.82944416f),Vector2f(35.79945321f,126.82938942f)}});
        vecAreas.push_back({pos:{Vector2f(35.79945321f,126.82938942f),Vector2f(35.79848795f,126.82944416f),Vector2f(35.79847674f,126.82904272f),Vector2f(35.79944128f,126.82898853f)}});
        vecAreas.push_back({pos:{Vector2f(35.79944128f,126.82898853f),Vector2f(35.79847674f,126.82904272f),Vector2f(35.79846408f,126.82863896f),Vector2f(35.79942790f,126.82858565f)}});
        vecAreas.push_back({pos:{Vector2f(35.79942790f,126.82858565f),Vector2f(35.79846408f,126.82863896f),Vector2f(35.79844890f,126.82823409f),Vector2f(35.79941480f,126.82818653f)}});
        vecAreas.push_back({pos:{Vector2f(35.79941480f,126.82818653f),Vector2f(35.79844890f,126.82823409f),Vector2f(35.79843661f,126.82783531f),Vector2f(35.79940124f,126.82778354f)}});
        vecAreas.push_back({pos:{Vector2f(35.79940124f,126.82778354f),Vector2f(35.79843661f,126.82783531f),Vector2f(35.79842222f,126.82742447f),Vector2f(35.79938785f,126.82737601f)}});
        vecAreas.push_back({pos:{Vector2f(35.79938785f,126.82737601f),Vector2f(35.79842222f,126.82742447f),Vector2f(35.79840850f,126.82703210f),Vector2f(35.79937412f,126.82698076f)}});
        vecAreas.push_back({pos:{Vector2f(35.79937412f,126.82698076f),Vector2f(35.79840850f,126.82703210f),Vector2f(35.79839726f,126.82662037f),Vector2f(35.79935865f,126.82656981f)}});
        vecAreas.push_back({pos:{Vector2f(35.79838381f,126.82623331f),Vector2f(35.79934564f,126.82617832f),Vector2f(35.79935865f,126.82656981f),Vector2f(35.79839726f,126.82662037f)}});
        vecAreas.push_back({pos:{Vector2f(35.79934564f,126.82617832f),Vector2f(35.79838381f,126.82623331f),Vector2f(35.79837005f,126.82582159f),Vector2f(35.79933253f,126.82577400f)}});
        vecAreas.push_back({pos:{Vector2f(35.79933253f,126.82577400f),Vector2f(35.79837005f,126.82582159f),Vector2f(35.79835674f,126.82540931f),Vector2f(35.79931876f,126.82535906f)}});
        vecAreas.push_back({pos:{Vector2f(35.79931876f,126.82535906f),Vector2f(35.79835674f,126.82540931f),Vector2f(35.79834361f,126.82499415f),Vector2f(35.79930401f,126.82494623f)}});
        vecAreas.push_back({pos:{Vector2f(35.79930401f,126.82494623f),Vector2f(35.79834361f,126.82499415f),Vector2f(35.79833012f,126.82458707f),Vector2f(35.79928719f,126.82454169f)}});
        vecAreas.push_back({pos:{Vector2f(35.79928719f,126.82454169f),Vector2f(35.79833012f,126.82458707f),Vector2f(35.79831610f,126.82418541f),Vector2f(35.79927281f,126.82414269f)}});
        vecAreas.push_back({pos:{Vector2f(35.79927281f,126.82414269f),Vector2f(35.79831610f,126.82418541f),Vector2f(35.79831032f,126.82399570f),Vector2f(35.79926693f,126.82394346f)}});
        vecAreas.push_back({pos:{Vector2f(35.79926693f,126.82394346f),Vector2f(35.79831032f,126.82399570f),Vector2f(35.79830317f,126.82379294f),Vector2f(35.79925897f,126.82373859f)}});
        vecAreas.push_back({pos:{Vector2f(35.79925897f,126.82373859f),Vector2f(35.79830317f,126.82379294f),Vector2f(35.79829059f,126.82339128f),Vector2f(35.79924405f,126.82334147f)}});
        vecAreas.push_back({pos:{Vector2f(35.79924405f,126.82334147f),Vector2f(35.79829059f,126.82339128f),Vector2f(35.79827568f,126.82299571f),Vector2f(35.79923256f,126.82294588f)}});
        vecAreas.push_back({pos:{Vector2f(35.79923256f,126.82294588f),Vector2f(35.79827568f,126.82299571f),Vector2f(35.79826526f,126.82259758f),Vector2f(35.79922484f,126.82254122f)}});
        vecAreas.push_back({pos:{Vector2f(35.80044104f,126.82974386f),Vector2f(35.79950695f,126.82978901f),Vector2f(35.79949286f,126.82938934f),Vector2f(35.80043208f,126.82934162f)}});
        vecAreas.push_back({pos:{Vector2f(35.80042467f,126.82913509f),Vector2f(35.79948662f,126.82918591f),Vector2f(35.79948021f,126.82898623f),Vector2f(35.80041772f,126.82894106f)}});
        vecAreas.push_back({pos:{Vector2f(35.80035454f,126.82713151f),Vector2f(35.79942100f,126.82717702f),Vector2f(35.79941476f,126.82697945f),Vector2f(35.80034659f,126.82693217f)}});
        vecAreas.push_back({pos:{Vector2f(35.80033266f,126.82651933f),Vector2f(35.79940209f,126.82656639f),Vector2f(35.79938971f,126.82617358f),Vector2f(35.80032037f,126.82612927f)}});
        vecAreas.push_back({pos:{Vector2f(35.80032037f,126.82612927f),Vector2f(35.79938971f,126.82617358f),Vector2f(35.79937479f,126.82576970f),Vector2f(35.80030762f,126.82572550f)}});
        vecAreas.push_back({pos:{Vector2f(35.79974002f,126.83793936f),Vector2f(35.79877367f,126.83798725f),Vector2f(35.79876065f,126.83755904f),Vector2f(35.79972943f,126.83751091f)}});
        vecAreas.push_back({pos:{Vector2f(35.79960254f,126.83831169f),Vector2f(35.79948953f,126.83858252f),Vector2f(35.79879236f,126.83861547f),Vector2f(35.79878480f,126.83835407f)}});
        vecAreas.push_back({pos:{Vector2f(35.79948953f,126.83858252f),Vector2f(35.79942782f,126.83873067f),Vector2f(35.79879716f,126.83876049f),Vector2f(35.79879236f,126.83861547f)}});
        vecAreas.push_back({pos:{Vector2f(35.79942782f,126.83873067f),Vector2f(35.79925785f,126.83913912f),Vector2f(35.79881068f,126.83916315f),Vector2f(35.79879716f,126.83876049f)}});
        vecAreas.push_back({pos:{Vector2f(35.79925785f,126.83913912f),Vector2f(35.79908609f,126.83955588f),Vector2f(35.79882176f,126.83956084f),Vector2f(35.79881068f,126.83916315f)}});
        vecAreas.push_back({pos:{Vector2f(35.80024450f,126.82368396f),Vector2f(35.79931043f,126.82373925f),Vector2f(35.79929578f,126.82334379f),Vector2f(35.80023077f,126.82329534f)}});
        vecAreas.push_back({pos:{Vector2f(35.80020127f,126.82248792f),Vector2f(35.79926746f,126.82253869f),Vector2f(35.79925497f,126.82213692f),Vector2f(35.80018707f,126.82209389f)}});
        vecAreas.push_back({pos:{Vector2f(35.80130375f,126.82204591f),Vector2f(35.80033660f,126.82210074f),Vector2f(35.80032366f,126.82170438f),Vector2f(35.80129091f,126.82165463f)}});
        vecAreas.push_back({pos:{Vector2f(35.80131606f,126.82244869f),Vector2f(35.80035171f,126.82250208f),Vector2f(35.80033660f,126.82210074f),Vector2f(35.80130375f,126.82204591f)}});
        vecAreas.push_back({pos:{Vector2f(35.80132838f,126.82285147f),Vector2f(35.80036294f,126.82290353f),Vector2f(35.80035171f,126.82250208f),Vector2f(35.80131606f,126.82244869f)}});
        vecAreas.push_back({pos:{Vector2f(35.80134249f,126.82325093f),Vector2f(35.80037588f,126.82330221f),Vector2f(35.80036294f,126.82290353f),Vector2f(35.80132838f,126.82285147f)}});
        vecAreas.push_back({pos:{Vector2f(35.80137009f,126.82405505f),Vector2f(35.80040374f,126.82410322f),Vector2f(35.80039958f,126.82397114f),Vector2f(35.80136493f,126.82392054f)}});
        vecAreas.push_back({pos:{Vector2f(35.80138384f,126.82446038f),Vector2f(35.80041893f,126.82450566f),Vector2f(35.80040374f,126.82410322f),Vector2f(35.80137009f,126.82405505f)}});
        vecAreas.push_back({pos:{Vector2f(35.80139651f,126.82486294f),Vector2f(35.80043134f,126.82491342f),Vector2f(35.80041893f,126.82450566f),Vector2f(35.80138384f,126.82446038f)}});
        vecAreas.push_back({pos:{Vector2f(35.80140321f,126.82506582f),Vector2f(35.80043849f,126.82511619f),Vector2f(35.80043134f,126.82491342f),Vector2f(35.80139651f,126.82486294f)}});
        vecAreas.push_back({pos:{Vector2f(35.80140981f,126.82526605f),Vector2f(35.80044564f,126.82532206f),Vector2f(35.80043849f,126.82511619f),Vector2f(35.80140321f,126.82506582f)}});
        vecAreas.push_back({pos:{Vector2f(35.80142364f,126.82566562f),Vector2f(35.80045757f,126.82571577f),Vector2f(35.80044564f,126.82532206f),Vector2f(35.80140981f,126.82526605f)}});
        vecAreas.push_back({pos:{Vector2f(35.79972943f,126.83751091f),Vector2f(35.79876065f,126.83755904f),Vector2f(35.79874794f,126.83716113f),Vector2f(35.79971708f,126.83710880f)}});
        vecAreas.push_back({pos:{Vector2f(35.79968996f,126.83626464f),Vector2f(35.79873515f,126.83631463f),Vector2f(35.79872189f,126.83591208f),Vector2f(35.79967634f,126.83586220f)}});
        vecAreas.push_back({pos:{Vector2f(35.79967634f,126.83586220f),Vector2f(35.79872189f,126.83591208f),Vector2f(35.79870818f,126.83551131f),Vector2f(35.79966326f,126.83546076f)}});
        vecAreas.push_back({pos:{Vector2f(35.79966326f,126.83546076f),Vector2f(35.79870818f,126.83551131f),Vector2f(35.79870158f,126.83530290f),Vector2f(35.79965541f,126.83525788f)}});
        vecAreas.push_back({pos:{Vector2f(35.79965541f,126.83525788f),Vector2f(35.79870158f,126.83530290f),Vector2f(35.79868733f,126.83490024f),Vector2f(35.79964178f,126.83485267f)}});
        vecAreas.push_back({pos:{Vector2f(35.79964178f,126.83485267f),Vector2f(35.79868733f,126.83490024f),Vector2f(35.79868065f,126.83469991f),Vector2f(35.79963529f,126.83465278f)}});
        vecAreas.push_back({pos:{Vector2f(35.79866730f,126.83430057f),Vector2f(35.79962220f,126.83425255f),Vector2f(35.79963529f,126.83465278f),Vector2f(35.79868065f,126.83469991f)}});
        vecAreas.push_back({pos:{Vector2f(35.79962220f,126.83425255f),Vector2f(35.79866730f,126.83430057f),Vector2f(35.79865313f,126.83389847f),Vector2f(35.79960749f,126.83385000f)}});
        vecAreas.push_back({pos:{Vector2f(35.79960749f,126.83385000f),Vector2f(35.79865313f,126.83389847f),Vector2f(35.79864058f,126.83349470f),Vector2f(35.79959449f,126.83344811f)}});
        vecAreas.push_back({pos:{Vector2f(35.79959449f,126.83344811f),Vector2f(35.79864058f,126.83349470f),Vector2f(35.79862659f,126.83309359f),Vector2f(35.79958086f,126.83304456f)}});
        vecAreas.push_back({pos:{Vector2f(35.79958086f,126.83304456f),Vector2f(35.79862659f,126.83309359f),Vector2f(35.79861368f,126.83269558f),Vector2f(35.79956697f,126.83264832f)}});
        vecAreas.push_back({pos:{Vector2f(35.79956697f,126.83264832f),Vector2f(35.79861368f,126.83269558f),Vector2f(35.79860086f,126.83229281f),Vector2f(35.79955378f,126.83224621f)}});
        vecAreas.push_back({pos:{Vector2f(35.80145164f,126.82651101f),Vector2f(35.80048683f,126.82656092f),Vector2f(35.80047247f,126.82616611f),Vector2f(35.80144026f,126.82611719f)}});
        vecAreas.push_back({pos:{Vector2f(35.80146565f,126.82691091f),Vector2f(35.80049995f,126.82696569f),Vector2f(35.80048683f,126.82656092f),Vector2f(35.80145164f,126.82651101f)}});
        vecAreas.push_back({pos:{Vector2f(35.80147875f,126.82730639f),Vector2f(35.80051278f,126.82735984f),Vector2f(35.80049995f,126.82696569f),Vector2f(35.80146565f,126.82691091f)}});
        vecAreas.push_back({pos:{Vector2f(35.80051278f,126.82735984f),Vector2f(35.80147875f,126.82730639f),Vector2f(35.80149260f,126.82771747f),Vector2f(35.80052680f,126.82776847f)}});
        vecAreas.push_back({pos:{Vector2f(35.80150525f,126.82811826f),Vector2f(35.80053881f,126.82816594f),Vector2f(35.80052680f,126.82776847f),Vector2f(35.80149260f,126.82771747f)}});
        vecAreas.push_back({pos:{Vector2f(35.80151791f,126.82851861f),Vector2f(35.80055544f,126.82856960f),Vector2f(35.80053881f,126.82816594f),Vector2f(35.80150525f,126.82811826f)}});
        vecAreas.push_back({pos:{Vector2f(35.80153146f,126.82892040f),Vector2f(35.80056656f,126.82897282f),Vector2f(35.80055544f,126.82856960f),Vector2f(35.80151791f,126.82851861f)}});
        vecAreas.push_back({pos:{Vector2f(35.80154447f,126.82932208f),Vector2f(35.80058183f,126.82937261f),Vector2f(35.80056656f,126.82897282f),Vector2f(35.80153146f,126.82892040f)}});
        vecAreas.push_back({pos:{Vector2f(35.80155731f,126.82972475f),Vector2f(35.80059357f,126.82977041f),Vector2f(35.80058183f,126.82937261f),Vector2f(35.80154447f,126.82932208f)}});
        vecAreas.push_back({pos:{Vector2f(35.80698912f,126.83942574f),Vector2f(35.80672537f,126.83966360f),Vector2f(35.80615681f,126.83869711f),Vector2f(35.80642723f,126.83846023f)}});
        vecAreas.push_back({pos:{Vector2f(35.80672537f,126.83966360f),Vector2f(35.80637402f,126.83996901f),Vector2f(35.80580826f,126.83900130f),Vector2f(35.80615681f,126.83869711f)}});
        vecAreas.push_back({pos:{Vector2f(35.80637402f,126.83996901f),Vector2f(35.80619497f,126.84012592f),Vector2f(35.80563092f,126.83915710f),Vector2f(35.80580826f,126.83900130f)}});
        vecAreas.push_back({pos:{Vector2f(35.80619497f,126.84012592f),Vector2f(35.80592409f,126.84035826f),Vector2f(35.80536284f,126.83939142f),Vector2f(35.80563092f,126.83915710f)}});
        vecAreas.push_back({pos:{Vector2f(35.80592409f,126.84035826f),Vector2f(35.80566151f,126.84059434f),Vector2f(35.80509423f,126.83962796f),Vector2f(35.80536284f,126.83939142f)}});
        vecAreas.push_back({pos:{Vector2f(35.80566151f,126.84059434f),Vector2f(35.80538379f,126.84082978f),Vector2f(35.80482741f,126.83986073f),Vector2f(35.80509423f,126.83962796f)}});
        vecAreas.push_back({pos:{Vector2f(35.80275247f,126.82372296f),Vector2f(35.80274825f,126.82372312f),Vector2f(35.80274164f,126.82352232f),Vector2f(35.80274765f,126.82352189f)}});
        vecAreas.push_back({pos:{Vector2f(35.80309777f,126.82430635f),Vector2f(35.80330768f,126.82431244f),Vector2f(35.80331814f,126.82434055f),Vector2f(35.80309839f,126.82432352f)}});
        vecAreas.push_back({pos:{Vector2f(35.80835899f,126.83826709f),Vector2f(35.80809857f,126.83849731f),Vector2f(35.80751545f,126.83750019f),Vector2f(35.80777983f,126.83726907f)}});
        vecAreas.push_back({pos:{Vector2f(35.80809857f,126.83849731f),Vector2f(35.80782572f,126.83873630f),Vector2f(35.80724774f,126.83773718f),Vector2f(35.80751545f,126.83750019f)}});
        vecAreas.push_back({pos:{Vector2f(35.80782572f,126.83873630f),Vector2f(35.80756395f,126.83896741f),Vector2f(35.80698075f,126.83797007f),Vector2f(35.80724774f,126.83773718f)}});
        vecAreas.push_back({pos:{Vector2f(35.80480853f,126.83983045f),Vector2f(35.80455764f,126.84004980f),Vector2f(35.80398267f,126.83906631f),Vector2f(35.80423617f,126.83884551f)}});
        vecAreas.push_back({pos:{Vector2f(35.80455764f,126.84004980f),Vector2f(35.80428596f,126.84028202f),Vector2f(35.80371163f,126.83930627f),Vector2f(35.80398267f,126.83906631f)}});
        vecAreas.push_back({pos:{Vector2f(35.80601696f,126.83561180f),Vector2f(35.80574898f,126.83584879f),Vector2f(35.80518463f,126.83488465f),Vector2f(35.80545297f,126.83464844f)}});
        vecAreas.push_back({pos:{Vector2f(35.80519259f,126.83633852f),Vector2f(35.80492470f,126.83657418f),Vector2f(35.80435819f,126.83560894f),Vector2f(35.80463031f,126.83537062f)}});
        vecAreas.push_back({pos:{Vector2f(35.80492470f,126.83657418f),Vector2f(35.80465924f,126.83680717f),Vector2f(35.80409733f,126.83583983f),Vector2f(35.80435819f,126.83560894f)}});
        vecAreas.push_back({pos:{Vector2f(35.80465924f,126.83680717f),Vector2f(35.80439116f,126.83704283f),Vector2f(35.80382899f,126.83607537f),Vector2f(35.80409733f,126.83583983f)}});
        vecAreas.push_back({pos:{Vector2f(35.80439116f,126.83704283f),Vector2f(35.80423399f,126.83718078f),Vector2f(35.80366965f,126.83621433f),Vector2f(35.80382899f,126.83607537f)}});
        vecAreas.push_back({pos:{Vector2f(35.80423399f,126.83718078f),Vector2f(35.80386177f,126.83750903f),Vector2f(35.80329860f,126.83653616f),Vector2f(35.80366965f,126.83621433f)}});
        vecAreas.push_back({pos:{Vector2f(35.80386177f,126.83750903f),Vector2f(35.80359064f,126.83774690f),Vector2f(35.80302756f,126.83677857f),Vector2f(35.80329860f,126.83653616f)}});
        vecAreas.push_back({pos:{Vector2f(35.80359064f,126.83774690f),Vector2f(35.80332769f,126.83797634f),Vector2f(35.80276265f,126.83701532f),Vector2f(35.80302756f,126.83677857f)}});
        vecAreas.push_back({pos:{Vector2f(35.80332769f,126.83797634f),Vector2f(35.80306034f,126.83821375f),Vector2f(35.80249422f,126.83725395f),Vector2f(35.80276265f,126.83701532f)}});
        vecAreas.push_back({pos:{Vector2f(35.80715542f,126.83619627f),Vector2f(35.80685206f,126.83645967f),Vector2f(35.80627162f,126.83545981f),Vector2f(35.80657435f,126.83519497f)}});
        vecAreas.push_back({pos:{Vector2f(35.80685206f,126.83645967f),Vector2f(35.80662467f,126.83665818f),Vector2f(35.80604595f,126.83565987f),Vector2f(35.80627162f,126.83545981f)}});
        vecAreas.push_back({pos:{Vector2f(35.80662467f,126.83665818f),Vector2f(35.80633239f,126.83691580f),Vector2f(35.80575250f,126.83591793f),Vector2f(35.80604595f,126.83565987f)}});
        vecAreas.push_back({pos:{Vector2f(35.80584053f,126.83734609f),Vector2f(35.80579444f,126.83738667f),Vector2f(35.80521798f,126.83638980f),Vector2f(35.80526407f,126.83634944f)}});
        vecAreas.push_back({pos:{Vector2f(35.80579444f,126.83738667f),Vector2f(35.80553384f,126.83761689f),Vector2f(35.80495369f,126.83662158f),Vector2f(35.80521798f,126.83638980f)}});
        vecAreas.push_back({pos:{Vector2f(35.80553384f,126.83761689f),Vector2f(35.80526540f,126.83785045f),Vector2f(35.80468571f,126.83685513f),Vector2f(35.80495369f,126.83662158f)}});
        vecAreas.push_back({pos:{Vector2f(35.80526540f,126.83785045f),Vector2f(35.80500021f,126.83808255f),Vector2f(35.80441682f,126.83708979f),Vector2f(35.80468571f,126.83685513f)}});
        vecAreas.push_back({pos:{Vector2f(35.80500021f,126.83808255f),Vector2f(35.80473564f,126.83831499f),Vector2f(35.80415416f,126.83732256f),Vector2f(35.80441682f,126.83708979f)}});
        vecAreas.push_back({pos:{Vector2f(35.80473564f,126.83831499f),Vector2f(35.80446379f,126.83855231f),Vector2f(35.80388869f,126.83755566f),Vector2f(35.80415416f,126.83732256f)}});
        vecAreas.push_back({pos:{Vector2f(35.80446379f,126.83855231f),Vector2f(35.80439024f,126.83861806f),Vector2f(35.80381470f,126.83762076f),Vector2f(35.80388869f,126.83755566f)}});
        vecAreas.push_back({pos:{Vector2f(35.80439024f,126.83861806f),Vector2f(35.80420058f,126.83878750f),Vector2f(35.80361612f,126.83779508f),Vector2f(35.80381470f,126.83762076f)}});
        vecAreas.push_back({pos:{Vector2f(35.80420058f,126.83878750f),Vector2f(35.80393628f,126.83901827f),Vector2f(35.80335687f,126.83802374f),Vector2f(35.80361612f,126.83779508f)}});
        vecAreas.push_back({pos:{Vector2f(35.80393628f,126.83901827f),Vector2f(35.80366173f,126.83926101f),Vector2f(35.80308213f,126.83826427f),Vector2f(35.80335687f,126.83802374f)}});
        vecAreas.push_back({pos:{Vector2f(35.80366173f,126.83926101f),Vector2f(35.80348997f,126.83941293f),Vector2f(35.80291299f,126.83841165f),Vector2f(35.80308213f,126.83826427f)}});
    }
#endif
}

bool ModeCNDN::init(bool ignore_checks)
{
    if (!copter.failsafe.radio)
    {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
    }
    else
    {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise position and desired velocity
    if (!pos_control->is_active_z())
    {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    init_speed();

    AP_Mission *_mission = AP::mission();
    if (vecPoints.empty()) {
        uint16_t nCmds = _mission->num_commands();
        for (uint16_t i=0; i < nCmds; i++) {
            AP_Mission::Mission_Command cmd;
            if (_mission->read_cmd_from_storage(i, cmd)) {
                if (cmd.id != MAV_CMD_DO_SET_ROI) continue;
                vecPoints.push_back(cmd.content.location);
            }
        }

        if (!vecPoints.empty()) {
            vecPoints.push_back(vecPoints.front());
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] AREA RECOVER FROM MISSION.");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] AREA RECOVER FAILED.");
        }
    }

    if (stage != RETURN_AUTO)
    {
        // initialise waypoint state
        stage = MANUAL;
        b_position_target = false;
        last_yaw_ms = 0;
        AP_Mission::mission_state mstate = _mission->state();
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MODE INITIALIZED[%d].", int(mstate));
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MISSION COMPLETE.");
    }

    return true;
}

void ModeCNDN::init_speed()
{
    wp_nav->set_speed_xy(_spd_xy_cmss.get()*1.0f);
    wp_nav->set_speed_up(_spd_up_cmss.get()*1.0f);
    wp_nav->set_speed_down(_spd_dn_cmss.get()*1.0f);
    wp_nav->wp_and_spline_init();
    pos_control->set_max_accel_xy(_acc_xy_cms.get()*1.0f);
    pos_control->calc_leash_length_xy();
}

void ModeCNDN::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-_spd_dn_cmss.get()*1.0f, _spd_up_cmss.get()*1.0f);
    pos_control->set_max_accel_z(g.pilot_accel_z);
    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -_spd_dn_cmss.get()*1.0f, _spd_up_cmss.get()*1.0f);

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (stage)
    {
    case RETURN_AUTO:
    case AUTO:
    {
        // if vehicle has reached destination switch to PREPARE_FINISH
        auto_control();
        if (reached_destination())
        {
            stage = PREPARE_FINISH;
            last_yaw_ms = 0;
            last_yaw_cd = copter.initial_armed_bearing;
            AP_Notify::events.waypoint_complete = 1;
            b_position_target = false;

            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] PREPARE FINISH.");

            Vector3f tpos;
            AP_Mission *_mission = AP::mission();
            uint16_t nCmds = _mission->num_commands();
            for (uint16_t i=1; i < nCmds; i++) {
                AP_Mission::Mission_Command cmd;
                if (_mission->read_cmd_from_storage(i, cmd)) {
                    if (cmd.id != MAV_CMD_NAV_WAYPOINT) continue;
                    if (cmd.content.location.get_vector_from_origin_NEU(tpos)) {
                        tpos.z = _mission_alt_cm.get() * 1.0f;
                        wp_nav->set_wp_destination(tpos, false);
                    } else {
                        return_to_manual_control(false);
                    }
                    break;
                }
            }
            auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
        }
    } break;

    case PREPARE_AUTO:
    case PREPARE_FINISH:
    {
        uint32_t now = AP_HAL::millis();
        auto_control();
        if (reached_destination())
        {
            if (last_yaw_ms == 0)
                last_yaw_ms = now;

            if ((now - last_yaw_ms) > 500)
            {
                last_yaw_ms = now;
                float dy = last_yaw_cd - ahrs.yaw_sensor;
                if (dy*dy < 1000.0f)
                {
                    if (stage == PREPARE_FINISH)
                    {
                        stage = FINISHED;
                        auto_yaw.set_mode(AUTO_YAW_HOLD);
                        AP_Notify::events.mission_complete = 1;
                        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] FINISHING.");
                    }
                    else if (stage == PREPARE_AUTO)
                    {
                        stage = AUTO;
                        init_speed();
                        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] GO WITH MISSIONS.");
                        copter.set_mode(Mode::Number::AUTO, ModeReason::RC_COMMAND);
                    }
                    
                }
            }
        }
    } break;

    case PREPARE_FOLLOW:
        processArea();
        if (stage == PREPARE_FOLLOW)
            return_to_manual_control(false);
        break;

    case EDGE_FOLLOW:
        processArea(1);
        if (stage == EDGE_FOLLOW)
            return_to_manual_control(false);
        break;
    
    case TAKE_AREA:
    case MOVE_TO_EDGE:
    case FINISHED:
    case MANUAL:
        manual_control();
        break;
    }
}

bool ModeCNDN::set_destination(const Vector3f &destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool yaw_relative)
{
    // ensure we are in position control mode
    if (stage != TAKE_AREA)
        pos_control_start();

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc))
    {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    return true;
}

#if 0
void ModeCNDN::live_log(const char *fmt, ...)
{
    uint32_t now = AP_HAL::millis();
    if (live_logt_ms == 0)
        live_logt_ms = now;

    if ((now - live_logt_ms) < 200)
        return;

    live_logt_ms = now;

    va_list args;
    char buff[128];
    va_start(args, fmt);
    vsprintf(buff, fmt, args);
    va_end(args);
    gcs().send_text(MAV_SEVERITY_INFO, "%s", buff);
}
#endif

// save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
void ModeCNDN::mission_command(uint8_t dest_num)
{
    // handle state machine changes
    switch (stage) {
    case MANUAL:
        if (_method.get() == 0)
            break;

        if (dest_num > 0) {
            wp_nav->wp_and_spline_init();
            init_speed();

            Vector3f stopping_point;
            wp_nav->get_wp_stopping_point(stopping_point);
            wp_nav->set_wp_destination(stopping_point, false);

            Location loc(copter.current_loc);
            gcs().send_cndn_trigger(loc.lat*1e-7, loc.lng*1e-7);
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] TRIGGER SEND.[%u,%u]", loc.lat, loc.lng);

            // detectEdge();
            // if (!vecPoints.empty()) {
            //     gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] %d DETECTED EDGES.", int(vecPoints.size()));
            //     stage = (dest_num==2) ? EDGE_FOLLOW : PREPARE_FOLLOW;
            // } else {
            //     gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] NOT REGISTERED AREA.");
            // }
        }
        break;

    case PREPARE_FOLLOW:
    case TAKE_AREA:
    case EDGE_FOLLOW:
    case MOVE_TO_EDGE:
    case PREPARE_AUTO:
    case AUTO:
    case RETURN_AUTO:
    case PREPARE_FINISH:
    case FINISHED:
    default:
        if (dest_num == 0) {
            wp_nav->wp_and_spline_init();
            return_to_manual_control(false);
            return;
        }
        break;
    }
}

// return manual control to the pilot
void ModeCNDN::return_to_manual_control(bool maintain_target)
{
    if (stage != MANUAL) {
        stage = MANUAL;
        b_position_target = false;
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest);
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        auto_yaw.set_mode(AUTO_YAW_HOLD);
        gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MANUAL CONTROL");
    }
}

void ModeCNDN::detectEdge()
{
    Vector3f hpos;
    vecPoints.clear();
    if (!ahrs.get_home().get_vector_from_origin_NEU(hpos))
        return;

    CNAREA edge;
    Location loc(copter.current_loc);
    bool bFound = false;
    for (uint16_t i=0; i<vecAreas.size(); i++) {
        CNAREA& area = vecAreas[i];
        if (!inside(area, loc))
            continue;
        edge = area;
        bFound = true;
        break;
    }

    if (!bFound)
        return;

    std::deque<Location> vecRects;
    for (int i = 0; i < 4; i++) {
        vecRects.push_back(Location(int32_t(edge.pos[i].x*1e7), int32_t(edge.pos[i].y*1e7), 300, Location::AltFrame::ABSOLUTE));
    }

    // from home position
    loc = Location(hpos);

    float minlen = vecRects.front().get_distance(loc);
    Location apos = vecRects.front();
    vecPoints.push_back(apos);
    for (int i = 1; i < (int)vecRects.size(); i++) {
        if (vecRects[i].get_distance(loc) < minlen) {
            apos = vecRects[i];
            minlen = apos.get_distance(loc);
        }
        vecPoints.push_back(vecRects[i]);
    }
    // rotate to nearest current pos
    for (int i = 0; i < (int)vecPoints.size(); i++) {
        if (vecPoints.front().get_distance(apos) <= 1e-3f)
            break;
        Location lpo = vecPoints.front();
        vecPoints.pop_front();
        vecPoints.push_back(lpo);
    }

    vecPoints.pop_front();
    if (vecPoints.front().get_distance(apos) < vecPoints.back().get_distance(apos))
        std::reverse(vecPoints.begin(), vecPoints.end());
    vecPoints.push_front(apos);
    vecPoints.push_back(apos);

    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] %f,%f", channel_pitch->get_control_in() * 500.0f, channel_roll->get_control_in() * 500.0f);
}

bool lineIntersection(const Vector3f& a,const Vector3f& b,const Vector3f& c,const Vector3f& d, Vector2f &o){
	Vector3f dmc(d - c);

	Vector3f bma(b - a);
	float det = bma.x*dmc.y - bma.y*dmc.x;
	if (fabsf(det) < 1e-9f)
		return false;

	Vector3f cma(c-a);
	float cdt = cma.x*dmc.y - cma.y*dmc.x;

    Vector3f oo(a + (bma * cdt/det));
    o.x = oo.x;
    o.y = oo.y;
    return true;
}

void ModeCNDN::processArea(int _mode)
{
    if (!AP::ahrs().home_is_set() || vecPoints.empty()) {
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] CAN NOT PROCESS AREA.");
        return_to_manual_control(false);
        return;
    }

    Vector3f vp0, vp1, vp2, vp3;
    if (!vecPoints[0].get_vector_from_origin_NEU(vp0)) {
        return_to_manual_control(false);
        return;
    }

    if (!vecPoints[1].get_vector_from_origin_NEU(vp1)) {
        return_to_manual_control(false);
        return;
    }

    if (!vecPoints[2].get_vector_from_origin_NEU(vp2)) {
        return_to_manual_control(false);
        return;
    }

    if (!vecPoints[3].get_vector_from_origin_NEU(vp3)) {
        return_to_manual_control(false);
        return;
    }

    Vector3f t1(vp1-vp0),t2(vp2-vp0);
    Vector3f nm(t1 % t2);
    nm.normalize();
    Rotation rtYaw = (nm.z >= 0) ? Rotation::ROTATION_YAW_90 : Rotation::ROTATION_YAW_270;

    Vector3f vr1(vp1 - vp0);
    Vector3f vr2(vp2 - vp1);
    Vector3f vr3(vp3 - vp2);
    Vector3f vr4(vp0 - vp3);
    vr1.z = vr2.z = vr3.z = vr4.z = 0;

    float l1 = vr1.length(), l2 = vr2.length(), l3 = vr3.length(), l4 = vr4.length();
    float vdl = MAX(l1, MAX(l2, MAX(l3, l4)));

    vr1.normalize();
    vr2.normalize();
    vr3.normalize();
    vr4.normalize();

    Vector3f vdn = vr1;

    vr1.rotate(rtYaw);
    vr2.rotate(rtYaw);
    vr3.rotate(rtYaw);
    vr4.rotate(rtYaw);

    float eg_cm = _dst_eg_cm.get() * 1.0f;
    Vector3f vp00 = vp0 + vr1 * eg_cm;
    Vector3f vp01 = vp1 + vr1 * eg_cm;
    Vector3f vp10 = vp1 + vr2 * eg_cm;
    Vector3f vp11 = vp2 + vr2 * eg_cm;
    Vector3f vp20 = vp2 + vr3 * eg_cm;
    Vector3f vp21 = vp3 + vr3 * eg_cm;
    Vector3f vp30 = vp3 + vr4 * eg_cm;
    Vector3f vp31 = vp0 + vr4 * eg_cm;

    Vector2f eg;
    if (lineIntersection(vp00,vp01,vp10,vp11,eg)) {
        vp10.x = eg.x;
        vp10.y = eg.y;
        vp01 = vp10;
    }

    if (lineIntersection(vp10,vp11,vp20,vp21,eg)) {
        vp20.x = eg.x;
        vp20.y = eg.y;
        vp11 = vp20;
    }

    if (lineIntersection(vp20,vp21,vp30,vp31,eg)) {
        vp30.x = eg.x;
        vp30.y = eg.y;
        vp21 = vp30;
    }

    if (lineIntersection(vp30,vp31,vp00,vp01,eg)) {
        vp00.x = eg.x;
        vp00.y = eg.y;
        vp31 = vp00;
    }

    // mission area
    float alt_cm = _mission_alt_cm.get()*1.0f;
    vp0 = vp01 = vp00;
    vp1 = vp11 = vp10;
    vp2 = vp21 = vp20;
    vp3 = vp31 = vp30;
    vp0.z = vp1.z = vp2.z = vp3.z = alt_cm;

    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] CREATING MISSION.");
    AP_Notify::events.waypoint_complete = 1;

    AP_Mission::Mission_Command cmd;

    AP::mission()->reset();
    AP::mission()->clear();

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = AP::ahrs().get_home();
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = AP::ahrs().get_home();
    cmd.content.location.set_alt_cm(_take_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
    AP::mission()->add_cmd(cmd);

    // create edge navigation
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = Location(vp0);
    cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 1;
    cmd.content.yaw.angle_deg = degNE(Vector2f(vp1.x,vp1.y), Vector2f(vp0.x,vp0.y));
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = Location(vp1);
    cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 1;
    cmd.content.yaw.angle_deg = degNE(Vector2f(vp2.x,vp2.y), Vector2f(vp1.x,vp1.y));
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = Location(vp2);
    cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 1;
    cmd.content.yaw.angle_deg = degNE(Vector2f(vp3.x,vp3.y),Vector2f(vp2.x,vp2.y));
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = Location(vp3);
    cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 1;
    cmd.content.yaw.angle_deg = degNE(Vector2f(vp0.x,vp0.y),Vector2f(vp3.x,vp3.y));
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = Location(vp0);
    cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 1;
    cmd.content.yaw.angle_deg = degNE(Vector2f(vp1.x,vp1.y),Vector2f(vp0.x,vp0.y));
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    float sw_cm = _spray_width_cm.get() * 1.0f;
    Vector3f step(vr1 * sw_cm);
    Vector3f p1, p2, p3, p4;

    CNAREA area;
    area.pos[0] = Vector2f(vp0.x,vp0.y);
    area.pos[1] = Vector2f(vp1.x,vp1.y);
    area.pos[2] = Vector2f(vp2.x,vp2.y);
    area.pos[3] = Vector2f(vp3.x,vp3.y);

    for (float l = 1.0f; l < 100.0f; l += 2.0f) {
        // check spray area
        vp01 = vp0 + step * (l + 0.5f);
        vp11 = vp1 + step * (l + 0.5f);
        if (!inside(area,Vector2f(vp01.x,vp01.y)) && !inside(area,Vector2f(vp11.x,vp11.y)))
            break;

        p1 = vp0 + step * l - vdn * vdl;
        p2 = vp1 + step * l + vdn * vdl;
        p1.z = p2.z = alt_cm;
        if (lineIntersection(p1,p2,vp1,vp2,eg)) {
            p2.x = eg.x;
            p2.y = eg.y;
        }
        if (lineIntersection(p1,p2,vp3,vp0,eg)) {
            p1.x = eg.x;
            p1.y = eg.y;
        }

        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 1;
        cmd.content.location = Location(p1);
        cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 1;
        cmd.content.location = Location(p2);
        cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
        AP::mission()->add_cmd(cmd);

        // check spray area
        vp01 = vp0 + step * (l + 1.5f);
        vp11 = vp1 + step * (l + 1.5f);
        if (!inside(area,Vector2f(vp01.x,vp01.y)) && !inside(area,Vector2f(vp11.x,vp11.y)))
            break;

        p4 = vp0 + step * (l + 1.0f) - vdn * vdl;
        p3 = vp1 + step * (l + 1.0f) + vdn * vdl;
        p3.z = p4.z = alt_cm;
        p1 = vp0 + step * l - vdn * vdl;
        p2 = vp1 + step * l + vdn * vdl;
        p1.z = p2.z = alt_cm;
        if (lineIntersection(p4,p3,vp1,vp2,eg)) {
            p3.x = eg.x;
            p3.y = eg.y;
        }
        if (lineIntersection(p4,p3,vp3,vp0,eg)) {
            p4.x = eg.x;
            p4.y = eg.y;
        }

        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 1;
        cmd.content.location = Location(p3);
        cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 1;
        cmd.content.location = Location(p4);
        cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
        AP::mission()->add_cmd(cmd);
    }

    // mission finish command.
    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.location = Location();
    cmd.content.relay.num = 255;
    cmd.content.relay.state = 1;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = vecPoints[0];
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = vecPoints[1];
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = vecPoints[2];
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = vecPoints[3];
    AP::mission()->add_cmd(cmd);

    wp_nav->wp_and_spline_init();

    if (!vecPoints.empty()) {
        Vector3f hpos(vp0.x, vp0.y, _mission_alt_cm.get() * 1.0f);
        wp_nav->set_wp_destination(hpos, false);
        last_yaw_cd = degNE(Vector2f(vp1.x,vp1.y), Vector2f(vp0.x,vp0.y)) * 100.0f;
        auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MOVE TO START POINT.");
        stage = PREPARE_AUTO;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] No edge detected.");
        return_to_manual_control(false);
    }
}

void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t packet;
        mavlink_msg_command_ack_decode(&msg, &packet);
        gcs().send_text(MAV_SEVERITY_INFO, "[ETRI] COMMAND_ACK(%d)", int(packet.command));
    } break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: { // MAV ID: 84
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode())
            break;

        Vector3f cpos = inertial_nav.get_position();

        bool bTargeted = false;
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_LOCAL_NED) {
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED) {
                packet.x = packet.y = 0.0f;
                packet.z = _take_alt_cm.get() * -0.01f;
                packet.z += cpos.z * 0.01f;
            }
            bTargeted = true;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
            packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
            break;
        }

        bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += cpos;
            } else {
                // convert from alt-above-home to alt-above-ekf-origin
                if (!AP::ahrs().home_is_set()) {
                    break;
                }
                Location origin;
                pos_vector.z += AP::ahrs().get_home().alt;
                if (copter.ahrs.get_origin(origin)) {
                    pos_vector.z -= origin.alt;
                }
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && vel_ignore && acc_ignore) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
            if (bTargeted) {
                b_position_target = true;
                //gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] SPT ON %0.3f,%0.3f,%0.3f", pos_vector.x, pos_vector.y, pos_vector.z);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] SPT ON.");
            }
        }
    } break;
    case MAVLINK_MSG_ID_CNDN_DETECT: {
        mavlink_cndn_detect_t packet;
        mavlink_msg_cndn_detect_decode(&msg, &packet);

        for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
                mavlink_channel_t chan_index = (mavlink_channel_t)(MAVLINK_COMM_0+i);
                if (HAVE_PAYLOAD_SPACE(chan_index, CNDN_REQUEST)) {
                    // we have space so send then clear that channel bit on the mask
                    mavlink_msg_cndn_request_send(chan_index, 1, 0, 0);
                }
        }
    } break;

    case MAVLINK_MSG_ID_CNDN_DATA: {
        mavlink_cndn_data_t packet;
        mavlink_msg_cndn_data_decode(&msg, &packet);
    } break;
    }
}

void ModeCNDN::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

void ModeCNDN::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // call z-axis position controller (wp_nav should have already updated its alt target)
    pos_control->update_z_controller();

    float roll_target = wp_nav->get_roll();
    float pitch_target = wp_nav->get_pitch();

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(roll_target, pitch_target, copter.aparm.angle_max);
#endif

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_target, pitch_target, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_target, pitch_target, auto_yaw.yaw(), true);
    }

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

void ModeCNDN::manual_control()
{
    copter.mode_loiter.run();
}

bool ModeCNDN::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        reach_wp_time_ms = 0;
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > CNDN_WP_RADIUS_CM) {
        reach_wp_time_ms = 0;
        return false;
    }

    // wait at least one second
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0)
        reach_wp_time_ms = now;

    return ((now - reach_wp_time_ms) > 1000);
}

void ModeCNDN::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

void ModeCNDN::return_to_mode()
{
    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] RELAY TO CNDN.");
    stage = RETURN_AUTO;
    copter.set_mode(Mode::Number::CNDN, ModeReason::MISSION_END);
}

void ModeCNDN::inject()
{
    if (copter.flightmode != this)
    {
        const float ofs_north = cosf(radians(ahrs.yaw_sensor)) * channel_pitch->get_control_in() * 0.001f;
        const float ofs_east  = sinf(radians(ahrs.yaw_sensor)) * channel_pitch->get_control_in() * 0.001f;
        AP::gps().set_offset_cm(ofs_north, ofs_east);
    }
    else
    {
        AP::gps().set_offset_cm(0, 0);
    }
}

#endif