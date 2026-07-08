#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <limits>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <joint_limit_table/JointLimitTable.h>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
  // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること

  // robotは、rootLinkがFreeJointでなければならない
public:
  // constant parameter
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Isometry3> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

  std::string chestLinkName; // wbmsでの腕の座標系基準フレーム

  std::vector<double> maxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for genRobot.

  const double g = 9.80665; // 重力加速度

public:
  // parameter
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > copOffset = std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >{cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3(0.0,0.02,0.0),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3(0.0,-0.02,0.0),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB)}; // 要素数2. rleg: 0. lleg: 1. endeffector frame. 足裏COPの目標位置. 幾何的な位置はcopOffset.value()無しで考えるが、目標COPを考えるときはcopOffset.value()を考慮する. クロスできたりジャンプできたりする脚でないと左右方向(外側向き)の着地位置修正は難しいので、その方向に転びそうになることが極力ないように内側にcopをオフセットさせておくとよい.  単位[m]. 滑らかに変化する.
  std::vector<std::vector<cnoid::Vector3> > legHull = std::vector<std::vector<cnoid::Vector3> >(2, std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.095,0.065,0.0),cnoid::Vector3(-0.095,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}); // 要素数2. rleg: 0. lleg: 1. endeffector frame.  凸形状で,上から見て半時計回り. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. 単位[m]. JAXONでは、COPがY -0.1近くにくるとギア飛びしやすいので、少しYの下限を少なくしている
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > defaultTranslatePos = std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(2,cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB)); // goPos, goVelocity, setFootSteps等するときの右脚と左脚の中心からの相対位置. また、reference frameとgenerate frameの対応付けに用いられる. (Z軸は鉛直). Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. RefToGenFrameConverter(handFixMode)が「左右」という概念を使うので、X成分も0でなければならない. 単位[m] 滑らかに変化する.
  std::vector<cpp_filters::TwoPointInterpolator<double> > isManualControlMode = std::vector<cpp_filters::TwoPointInterpolator<double> >(2, cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::LINEAR)); // 要素数2. 0: rleg. 1: lleg. 0~1. 連続的に変化する. 1ならicEETargetPoseに従い、refEEWrenchに応じて重心をオフセットする. 0ならImpedanceControlをせず、refEEWrenchを無視し、DampingControlを行わない. 静止状態で無い場合や、支持脚の場合は、勝手に0になる. 両足が同時に1になることはない. 1にするなら、RefToGenFrameConverter.refFootOriginWeightを0にしたほうが良い. 1の状態でStartAutoStabilizerすると、遊脚で始まる

  std::vector<bool> jointControllable; // 要素数と順序はnumJoints()と同じ. falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). IKでは動かさない(ref値をそのまま). トルク計算では目標トルクを通常通り計算する. このパラメータはMODE_IDLEのときにしか変更されない

public:
  // from data port
  cnoid::BodyPtr refRobotRaw; // reference. reference world frame
  std::vector<cnoid::Vector6> refEEWrenchOrigin; // 要素数と順序はeeNameと同じ.FootOrigin frame. EndEffector origin. ロボットが受ける力
  std::vector<cpp_filters::TwoPointInterpolatorSE3> refEEPoseRaw; // 要素数と順序はeeNameと同じ. reference world frame. EEPoseはjoint angleなどと比べて遅い周期で届くことが多いので、interpolaterで補間する.
  cnoid::BodyPtr actRobotRaw; // actual. actual imu world frame
  class Collision {
  public:
    std::string link1 = "";
    cnoid::Vector3 point1 = cnoid::Vector3::Zero(); // link1 frame
    std::string link2 = "";
    cnoid::Vector3 point2 = cnoid::Vector3::Zero(); // link2 frame
    cnoid::Vector3 direction21 = cnoid::Vector3::UnitX(); // generate frame
    double distance = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<Collision> selfCollision;
  std::vector<std::vector<cnoid::Vector3> > steppableRegion; // generate frame. 着地可能領域の凸包の集合. 要素数0なら、全ての領域が着地可能として扱われる. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. 
  std::vector<double> steppableHeight; // generate frame. 要素数と順序はsteppableRegionと同じ。steppableRegionの各要素の重心Z.
  double relLandingHeight = -1e15; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ. -1e10未満なら、relLandingHeightとrelLandingNormalは無視される. footStepNodesListsの次のnodeに移るたびにFootStepGeneratorによって-1e15に上書きされる.
  cnoid::Vector3 relLandingNormal = cnoid::Vector3::UnitZ(); // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ軸の方向. ノルムは常に1
public:
  // AutoStabilizerの中で計算更新される.

  // refToGenFrameConverter
  cnoid::BodyPtr refRobot; // reference. generate frame
  std::vector<cnoid::Isometry3> refEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> refEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力
  double refdz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = cpp_filters::TwoPointInterpolatorSE3(cnoid::Isometry3::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // generate frame. Z軸は鉛直. 支持脚の位置姿勢(Z軸は鉛直)にdefaultTranslatePosを適用したものの間をつなぐ. interpolatorによって連続的に変化する. reference frameとgenerate frameの対応付けに用いられる
  bool isWbmsAbsolute = false; // 操縦時の姿勢反映モード
  std::vector<double> humanToRobotRatio; // 操縦時のエンドエフェクタ位置の拡大率
  std::vector<cnoid::Isometry3> wbmsOffsetPoseMaster; // 要素数と順序はeeNameと同じ. wbms起動時の姿勢を保存
  std::vector<cnoid::Isometry3> wbmsOffsetPoseSlave; // 要素数と順序はeeNameと同じ. wbms起動時の姿勢を保存


  // actToGenFrameConverter
  cnoid::BodyPtr actRobot; // actual. generate frame
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>(3.5, cnoid::Vector3::Zero());  // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ
  std::vector<cnoid::Isometry3> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> actEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力

  // ExternalForceHandler
  double omega = std::sqrt(g / refdz); // DCMの計算に用いる. 0より大きい
  cnoid::Vector3 l = cnoid::Vector3(0, 0, refdz); // generate frame. FootGuidedControlで外力を計算するときの、ZMP-重心の相対位置に対するオフセット. また、CMPの計算時にDCMに対するオフセット(CMP + l = DCM). 連続的に変化する.
  cnoid::Vector3 sbpOffset = cnoid::Vector3::Zero(); // generate frame. 外力考慮重心と重心のオフセット. genCog = genRobot->centerOfMass() - sbpOffset. actCog = actRobot->centerOfMass() - sbpOffset.
  cnoid::Vector3 actCog; // generate frame. 現在のCOMにsbpOffsetを施したもの actCog = actRobot->centerOfMass() - sbpOffset

  // ImpedanceController
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > icEEOffset; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. icで計算されるオフセット
  std::vector<cnoid::Isometry3> icEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. icで計算された目標位置姿勢. icEETargetPose = icEEOffset + refEEPose

  // CmdVelGenerator
  cnoid::Vector3 cmdVel = cnoid::Vector3::Zero(); // X[m/s] Y[m/s] theta[rad/s]. Z軸はgenerate frame鉛直. support leg frame. 不連続に変化する

  // FootStepGenerator
  class FootStepNodes {
  public:
    /*
      各足につきそれぞれ、remainTime 後に dstCoordsに動く.

      footstepNodesList[0]のisSupportPhaseは、変更されない
      footstepNodesList[0]のdstCoordsを変更する場合には、footstepNodesList[0]のremainTimeの小ささに応じて変更量を小さくする. remainTimeがほぼゼロなら変更量もほぼゼロ.
      footstepNodesList[0]は、!footstepNodesList[0].isSupportPhase && footstepNodesList[1].isSupportPhaseの足がある場合に、突然footstepNodesList[1]に遷移する場合がある(footStepGeneratorのearlyTouchDown)
      それ以外には、footstepNodesList[0]のremainTimeが突然0になることはない
      footstepNodesList[1]のisSupportPhaseは、footstepNodesListのサイズが1である場合を除いて変更されない.
      両足支持期の次のstepのisSupportPhaseを変えたり、後ろに新たにfootstepNodesを追加する場合、必ず両足支持期のremainTimeをそれなりに長い時間にする(片足に重心やrefZmpを移す補間の時間のため)

      右脚支持期の直前のnodeのendRefZmpStateはRLEGでなければならない
      右脚支持期のnodeのendRefZmpStateはRLEGでなければならない
      左脚支持期の直前のnodeのendRefZmpStateはLLEGでなければならない
      左脚支持期のnodeのendRefZmpStateはLLEGでなければならない
      両脚支持期の直前のnodeのendRefZmpStateはRLEG or LLEG or MIDDLEでなければならない.
      両脚支持期のnodeのendRefZmpStateはRLEG or LLEG or MIDDLEでなければならない.
      footstepNodesList[0]のendRefZmpStateは変更されない.
      footstepNodesList[0]のendRefZmpStateは、isStatic()である場合を除いて変更されない.
    */
    std::vector<cnoid::Isometry3> dstCoords = std::vector<cnoid::Isometry3>(NUM_LEGS,cnoid::Isometry3::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame.
    std::vector<bool> isSupportPhase = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. footstepNodesListの末尾の要素が両方falseであることは無い
    double remainTime = 0.0;
    enum class refZmpState_enum{RLEG, LLEG, MIDDLE};
    refZmpState_enum endRefZmpState = refZmpState_enum::MIDDLE; // このnode終了時のrefZmpの位置.

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(NUM_LEGS,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する
    std::vector<bool> stopCurrentPosition = std::vector<bool>(NUM_LEGS, false); // 現在の位置で強制的に止めるかどうか
    std::vector<double> goalOffset = std::vector<double>(NUM_LEGS,0.0); // [m]. 遊脚軌道生成時に、遅づきの場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセットの最大値. 0以下.
    std::vector<double> touchVel = std::vector<double>(NUM_LEGS,0.3); // 0より大きい. 単位[m/s]. 足を下ろすときの速さ

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<FootStepNodes> footstepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
  std::vector<cnoid::Isometry3> srcCoords = std::vector<cnoid::Isometry3>(NUM_LEGS,cnoid::Isometry3::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在のfootstep開始時のgenCoords
  std::vector<cnoid::Isometry3> dstCoordsOrg = std::vector<cnoid::Isometry3>(NUM_LEGS,cnoid::Isometry3::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在のfootstep開始時のdstCoords
  double remainTimeOrg = 0.0; // 現在のfootstep開始時のremainTime
  enum SwingState_enum{LIFT_PHASE, SWING_PHASE, DOWN_PHASE};
  std::vector<SwingState_enum> swingState = std::vector<SwingState_enum>(NUM_LEGS,LIFT_PHASE); // 要素数2. rleg: 0. lleg: 0. isSupportPhase = falseの脚は、footstep開始時はLIFT_PHASEで、LIFT_PHASE->SWING_PHASE->DOWN_PHASEと遷移する. 一度DOWN_PHASEになったら次のfootstepが始まるまで別のPHASEになることはない. DOWN_PHASEのときはfootstepNodesList[0]のdstCoordsはgenCoordsよりも高い位置に変更されることはない. isSupportPhase = trueの脚は、swingStateは参照されない(常にLIFT_PHASEとなる).
  double elapsedTime = 0.0; // 現在のfootstep開始時からの経過時間
  std::vector<bool> prevSupportPhase = std::vector<bool>{true, true}; // 要素数2. rleg: 0. lleg: 1. 一つ前の周期でSupportPhaseだったかどうか

  // LegCoordsGenerator
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = std::vector<cpp_filters::TwoPointInterpolatorSE3>(NUM_LEGS, cpp_filters::TwoPointInterpolatorSE3(cnoid::Isometry3::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB)); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = {footguidedcontroller::LinearTrajectory<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),0.0)}; // 要素数1以上. generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp軌道

  cnoid::Vector3 genCog; // generate frame. abcで計算された目標COM
  cnoid::Vector3 genCogVel;  // generate frame.  abcで計算された目標COM速度
  cnoid::Vector3 genCogAcc;  // generate frame.  abcで計算された目標COM加速度
  cnoid::Vector3 wbmsNominalGenCogBeforeWbmsIntegration = cnoid::Vector3::Zero(); // generate frame. WBMS統合前にLegCoordsGeneratorが生成した通常歩行用COM
  bool wbmsNominalGenCogBeforeWbmsIntegrationValid = false;
  std::vector<cnoid::Isometry3> abcEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. abcで計算された目標位置姿勢

  // Stabilizer
  cpp_filters::TwoPointInterpolator<cnoid::Vector3> stOffsetRootRpy = cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::LINEAR);; // gaitParam.footMidCoords座標系. stで計算された目標位置姿勢オフセット
  cnoid::Isometry3 stTargetRootPose = cnoid::Isometry3::Identity(); // generate frame. stTargetRootPose = stOffsetRootRpy + refRobot->rootLink
  cnoid::Vector3 stTargetZmp; // generate frame. stで計算された目標ZMP
  std::vector<cnoid::Vector6> stEETargetWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力
  std::vector<cpp_filters::TwoPointInterpolator<double> > stServoPGainPercentage; // 要素数と順序はrobot->numJoints()と同じ. 0~100. 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻する
  std::vector<cpp_filters::TwoPointInterpolator<double> > stServoDGainPercentage; // 要素数と順序はrobot->numJoints()と同じ. 0~100. 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻する
  cnoid::BodyPtr actRobotTqc; // output. 関節トルク制御用. (actRobotと同じだが、uの値として指令関節トルクが入っている)

  // FullbodyIKSolver
  cnoid::BodyPtr genRobot; // output. 関節位置制御用
  cpp_filters::TwoPointInterpolator<double> wbmsMode = cpp_filters::TwoPointInterpolator<double>(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  // WBMS
  double wbmsInterpolateDuration = 0.3; // 操縦指令の補間時間
  double wbmsWalkingStabilityStartTime = 2.0; // [s]. WBMS中に歩行開始するとき、歩行開始前にroot姿勢を復帰させる時間
  double wbmsWalkingStabilityStopTime = 1.0; // [s]. WBMS中に静止へ戻ったとき、root姿勢を弱める補間時間
  double wbmsWalkingPreparationTimeout = 6.0; // [s]. 歩行準備遷移のtimeout
  double wbmsWalkingPreparationReturnTime = 1.0; // [s]. CHEST/COM XYを歩行可能基準へ戻す時間
  double wbmsWalkingPreparationHandoffTime = 0.5; // [s]. WBMS姿勢拘束を通常歩行側へ渡す時間
  double wbmsWalkingPreparationSettleTime = 0.1; // [s]. READY条件の連続成立時間
  double wbmsWalkingPreparationVelocityEps = 1e-3; // [m/s or rad/s]. applied command収束判定
  double wbmsWalkingPreparationChestErrorEps = 0.03; // [rad]
  double wbmsWalkingPreparationComXYErrorEps = 0.01; // [m]
  double wbmsWalkingPreparationComZErrorEps = 0.01; // [m]
  double wbmsWalkingPreparationRootErrorEps = 0.01; // [rad]
  double wbmsWalkingPreparationMaxJointDeltaEps = 0.08; // [rad or m]
  cnoid::Vector3 wbmsWalkingPreparationTorsoAngularVelocityLimit = cnoid::Vector3(0.10, 0.10, 0.20); // [rad/s]. 歩行準備RETURN用CHEST角速度limit
  cnoid::Vector3 wbmsWalkingPreparationTorsoAngularAccelerationLimit = cnoid::Vector3(0.30, 0.30, 0.60); // [rad/s^2]. 歩行準備RETURN用CHEST角加速度limit
  cnoid::Vector3 wbmsWalkingPreparationComVelocityLimit = cnoid::Vector3(0.03, 0.03, 0.03); // [m/s]. 歩行準備RETURN用COM速度limit
  cnoid::Vector3 wbmsWalkingPreparationComAccelerationLimit = cnoid::Vector3(0.10, 0.10, 0.10); // [m/s^2]. 歩行準備RETURN用COM加速度limit
  bool isWbmsWalkingStartDelay = false; // WBMS中の歩行開始前に姿勢復帰待ちをしている
  double wbmsWalkingStartDelayRemainTime = 0.0; // [s]. WBMS中の歩行開始前姿勢復帰待ちの残り時間
  double wbmsVelocityCommandTimeout = 0.2; // [s]. refTorsoVelInを最後に受信してから速度指令を無効にするまでの時間
  cnoid::Vector3 wbmsTorsoAngularVelocityLimit = cnoid::Vector3(0.15, 0.15, 0.3); // [rad/s]. CHEST角速度指令のlimit
  cnoid::Vector3 wbmsTorsoAngularAccelerationLimit = cnoid::Vector3(0.5, 0.5, 1.0); // [rad/s^2]. CHEST角速度指令の加速度limit
  cnoid::Vector3 wbmsTorsoRpyLowerLimit = cnoid::Vector3(-0.10, -0.02, -0.30); // [rad]. WBMS開始時CHEST姿勢からの実現姿勢差分下限
  cnoid::Vector3 wbmsTorsoRpyUpperLimit = cnoid::Vector3(0.10, 0.25, 0.30); // [rad]. WBMS開始時CHEST姿勢からの実現姿勢差分上限
  cnoid::Vector3 wbmsTorsoOrientationWeight = cnoid::Vector3(0.3, 0.3, 0.3); // WBMS CHEST姿勢IKの姿勢weight
  cnoid::Vector3 wbmsTorsoOrientationMaxError = cnoid::Vector3(0.15, 0.15, 0.30); // [rad/s]. WBMS CHEST姿勢IKの1秒あたりの最大姿勢補正量
  cnoid::Vector3 wbmsComVelocityLimit = cnoid::Vector3(0.05, 0.05, 0.05); // [m/s]. COM速度limit
  cnoid::Vector3 wbmsComAccelerationLimit = cnoid::Vector3(0.2, 0.2, 0.2); // [m/s^2]. COM速度指令の加速度limit
  cnoid::Vector3 wbmsComOffsetLowerLimit = cnoid::Vector3(-0.10, -0.08, -0.20); // [m]. WBMS開始時COMからの位置差分下限。footMidCoords座標系
  cnoid::Vector3 wbmsComOffsetUpperLimit = cnoid::Vector3(0.10, 0.08, 0.05); // [m]. WBMS開始時COMからの位置差分上限。footMidCoords座標系
  cnoid::Vector3 wbmsComPositionWeight = cnoid::Vector3(3.0, 3.0, 1.0); // WBMS COM位置IKのweight
  double wbmsComXYSupportMargin = 0.03; // [m]. 支持多角形境界から内側へ確保するCOM/ZMP XY余裕

  cnoid::Vector3 wbmsRawComVelocityCommand = cnoid::Vector3::Zero(); // [m/s]. refTorsoVelIn vx/vy/vzから保持したCOM速度指令。footMidCoords座標系
  cnoid::Vector3 wbmsRawTorsoAngularVelocityCommand = cnoid::Vector3::Zero(); // [rad/s]. refTorsoVelIn vr/vp/vaから保持したCHEST角速度指令。footMidCoords軸
  cnoid::Vector3 wbmsAppliedComVelocityCommand = cnoid::Vector3::Zero(); // [m/s]. 加速度limit通過後のCOM速度指令
  cnoid::Vector3 wbmsAppliedTorsoAngularVelocityCommand = cnoid::Vector3::Zero(); // [rad/s]. 加速度limit通過後のCHEST角速度指令
  double wbmsVelocityCommandAge = 1.0; // [s]. 最後にfiniteなrefTorsoVelInを受信してからの経過時間
  bool wbmsVelocityCommandValid = false;

  cnoid::Matrix3 wbmsStartChestRInFootMid = cnoid::Matrix3::Identity(); // WBMS開始時CHEST姿勢。footMidCoords座標系
  cnoid::Vector3 wbmsStartComInFootMid = cnoid::Vector3::Zero(); // [m]. WBMS開始時robot COM。footMidCoords座標系
  bool wbmsPostureBaselineValid = false;

  std::vector<double> wbmsPostureReferenceQ; // 投影IK後の関節角参照。M1では旧IKへ接続しない
  std::vector<bool> wbmsPostureReferenceJointMask; // 投影IKのvariableに含まれる関節。falseの関節は最終IKで従来referenceを使う
  cnoid::Matrix3 wbmsProjectedChestR = cnoid::Matrix3::Identity(); // 投影後CHEST姿勢。generate frame
  cnoid::Vector3 wbmsProjectedRobotCom = cnoid::Vector3::Zero(); // 投影後robot COM。generate frame
  cnoid::Vector3 wbmsRealizedComVelocity = cnoid::Vector3::Zero(); // [m/s]. 投影後に実現したCOM速度
  cnoid::Vector3 wbmsRealizedTorsoAngularVelocity = cnoid::Vector3::Zero(); // [rad/s]. 投影後に実現したCHEST角速度
  bool wbmsPostureReferenceValid = false;
  double wbmsWalkingStabilityModeValue = 0.0; // 0〜1。歩行中および歩行開始遅延中に1へ近づく
  double wbmsOperationModeValue = 0.0; // wbmsMode * (1 - walkingStabilityMode)

  enum WbmsWalkingPreparationPhase {
    WBMS_WALKING_PREPARATION_INACTIVE = 0,
    WBMS_WALKING_PREPARATION_REQUESTED = 1,
    WBMS_WALKING_PREPARATION_DECELERATING = 2,
    WBMS_WALKING_PREPARATION_RETURNING = 3,
    WBMS_WALKING_PREPARATION_HANDOFF = 4,
    WBMS_WALKING_PREPARATION_READY = 5,
    WBMS_WALKING_PREPARATION_WALKING_HOLD = 6,
    WBMS_WALKING_PREPARATION_FAILED = 7
  };

  enum WbmsWalkingPreparationFailureCode {
    WBMS_WALKING_PREPARATION_FAILURE_NONE = 0,
    WBMS_WALKING_PREPARATION_FAILURE_SNAPSHOT = 1,
    WBMS_WALKING_PREPARATION_FAILURE_TIMEOUT = 2,
    WBMS_WALKING_PREPARATION_FAILURE_NONFINITE = 3,
    WBMS_WALKING_PREPARATION_FAILURE_UNSAFE = 4,
    WBMS_WALKING_PREPARATION_FAILURE_CANCELLED = 5
  };

  WbmsWalkingPreparationPhase wbmsWalkingPreparationPhase = WBMS_WALKING_PREPARATION_INACTIVE;
  WbmsWalkingPreparationFailureCode wbmsWalkingPreparationFailureCode = WBMS_WALKING_PREPARATION_FAILURE_NONE;
  bool wbmsWalkingPreparationReleaseRequested = false; // READY成立後、次周期冒頭releaseするためのフラグ
  bool wbmsWalkingPreparationSnapshotValid = false;
  bool wbmsWalkingComHeightHoldValid = false;
  double wbmsWalkingPreparationElapsedTime = 0.0; // [s]
  double wbmsWalkingPreparationPhaseElapsedTime = 0.0; // [s]
  double wbmsWalkingPreparationSettleElapsedTime = 0.0; // [s]
  double wbmsWalkingPreparationReturnAlpha = 0.0;
  double wbmsWalkingPreparationHandoffAlpha = 0.0;
  double heldRobotComHeightInFootMid = 0.0; // [m]. 歩行指令受付後snapshotしたfootMid基準robot COM高さ
  cnoid::Matrix3 wbmsWalkingPreparationStartChestRInFootMid = cnoid::Matrix3::Identity();
  cnoid::Vector3 wbmsWalkingPreparationStartRobotComInFootMid = cnoid::Vector3::Zero();
  cnoid::Vector3 wbmsWalkingPreparationNominalRobotComInFootMid = cnoid::Vector3::Zero();
  cnoid::Matrix3 wbmsWalkingPreparationStartRootR = cnoid::Matrix3::Identity();
  cnoid::Matrix3 wbmsWalkingPreparationTargetRootR = cnoid::Matrix3::Identity();
  cnoid::Matrix3 wbmsWalkingPreparationTargetChestRInFootMid = cnoid::Matrix3::Identity();
  cnoid::Vector3 wbmsWalkingPreparationTargetRobotComInFootMid = cnoid::Vector3::Zero();
  cnoid::Vector3 wbmsWalkingPreparationReturnTorsoAngularVelocity = cnoid::Vector3::Zero();
  cnoid::Vector3 wbmsWalkingPreparationReturnRootAngularVelocity = cnoid::Vector3::Zero();
  cnoid::Vector3 wbmsWalkingPreparationReturnComVelocity = cnoid::Vector3::Zero();
  double wbmsWalkingPreparationChestError = 0.0;
  double wbmsWalkingPreparationComXYError = 0.0;
  double wbmsWalkingPreparationComZError = 0.0;
  double wbmsWalkingPreparationRootError = 0.0;

  enum WbmsProjectionStatus {
    WBMS_PROJECTION_NOT_RUN = 0,
    WBMS_PROJECTION_DISABLED = 1,
    WBMS_PROJECTION_BASELINE_INVALID = 2,
    WBMS_PROJECTION_NO_VARIABLE = 3,
    WBMS_PROJECTION_TARGET_INVALID = 4,
    WBMS_PROJECTION_SUPPORT_HULL_INVALID = 5,

    WBMS_PROJECTION_INVALID_NONFINITE = 10,
    WBMS_PROJECTION_INVALID_JOINT_LIMIT = 11,
    WBMS_PROJECTION_INVALID_JOINT_STEP = 12,
    WBMS_PROJECTION_INVALID_ROOT_TRANSLATION = 13,
    WBMS_PROJECTION_INVALID_ROOT_ROTATION = 14,
    WBMS_PROJECTION_INVALID_FOOT_POSITION = 15,
    WBMS_PROJECTION_INVALID_FOOT_ROTATION = 16,

    WBMS_PROJECTION_VALID_IDLE = 20,
    WBMS_PROJECTION_VALID_ACTIVE = 21,
    WBMS_PROJECTION_VALID_BLOCKED = 22
  };

  WbmsProjectionStatus wbmsProjectionStatus = WBMS_PROJECTION_NOT_RUN;
  bool wbmsProjectionAllConstraintsSatisfied = false;
  bool wbmsProjectionCandidateSafe = false;
  bool wbmsProjectionSupportHullValid = false;
  double wbmsProjectionRootTranslationStep = 0.0; // [m]
  double wbmsProjectionRootRotationStep = 0.0; // [rad]
  double wbmsProjectionMaxJointStep = 0.0; // [rad or m]
  double wbmsProjectionMinJointLimitMargin = std::numeric_limits<double>::max();
  double wbmsProjectionMaxFootPositionError = 0.0; // [m]
  double wbmsProjectionMaxFootRotationError = 0.0; // [rad]

  // for debug data
  class DebugData {
  public:
    std::vector<cnoid::Vector3> strideLimitationHull = std::vector<cnoid::Vector3>(); // generate frame. overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる). Z成分には0を入れる
    std::vector<std::vector<cnoid::Vector3> > capturableHulls = std::vector<std::vector<cnoid::Vector3> >(); // generate frame. 要素数と順番はcandidatesに対応
    std::vector<double> cpViewerLog = std::vector<double>(37, 0.0);
    double wbmsProjectorTime = 0.0; // [s]. WbmsPostureControl::procの計算時間
    double wbmsFinalIKTime = 0.0; // [s]. FullbodyIKSolver::solveFullbodyIKの計算時間
    double onExecuteTime = 0.0; // [s]. AutoStabilizer::onExecuteの計算時間
    double wbmsProjectorQpSignatureHitDelta = 0.0; // M5.9一時計測. projector QP workspace signature hit数の周期増分
    double wbmsProjectorQpSignatureMissDelta = 0.0; // M5.9一時計測. projector QP workspace signature miss数の周期増分
    double wbmsProjectorQpInitializeDelta = 0.0; // M5.9一時計測. projector QP solver initialize回数の周期増分
    double wbmsProjectorQpUpdateFailureDelta = 0.0; // M5.9一時計測. projector QP update失敗回数の周期増分
    double wbmsProjectorQpSolveFailureDelta = 0.0; // M5.9一時計測. projector QP solve失敗回数の周期増分
    double wbmsFinalIKQpSignatureHitDelta = 0.0; // M5.9一時計測. final IK QP workspace signature hit数の周期増分
    double wbmsFinalIKQpSignatureMissDelta = 0.0; // M5.9一時計測. final IK QP workspace signature miss数の周期増分
    double wbmsFinalIKQpInitializeDelta = 0.0; // M5.9一時計測. final IK QP solver initialize回数の周期増分
    double wbmsFinalIKQpUpdateFailureDelta = 0.0; // M5.9一時計測. final IK QP update失敗回数の周期増分
    double wbmsFinalIKQpSolveFailureDelta = 0.0; // M5.9一時計測. final IK QP solve失敗回数の周期増分
    cnoid::Vector3 wbmsFinalIKRealizedComVelocity = cnoid::Vector3::Zero(); // [m/s]. final IK後robot COMの実現速度
    cnoid::Vector3 wbmsFinalIKRealizedChestAngularVelocity = cnoid::Vector3::Zero(); // [rad/s]. final IK後CHEST姿勢の実現角速度
    cnoid::Vector3 wbmsFinalIKPreviousRobotComInFootMid = cnoid::Vector3::Zero();
    cnoid::Matrix3 wbmsFinalIKPreviousChestRInFootMid = cnoid::Matrix3::Identity();
    std::vector<double> wbmsFinalIKPreviousJointQ;
    bool wbmsFinalIKPreviousValid = false;
    double wbmsFinalIKMaxJointDelta = 0.0; // [rad or m]. final IK後関節角の一周期最大変化量
    bool wbmsWalkingPendingCommandReleaseEvent = false; // 現周期でlegacy delayのpending commandをreleaseした
    bool wbmsWalkingApiRejectedNotReadyEvent = false; // WBMS歩行準備未完了により歩行APIをrejectした
    bool wbmsWalkingApiAcceptedReadyEvent = false; // WBMS歩行準備READY後に歩行APIを受け付けた
    bool wbmsWalkingPreparationStartEvent = false; // startWbmsWalkingPreparation serviceが受理された
    bool wbmsWalkingPreparationCancelEvent = false; // cancelWbmsWalkingPreparation serviceが受理された

    void resetWbmsFinalIKDiagnostics(){
      wbmsFinalIKRealizedComVelocity.setZero();
      wbmsFinalIKRealizedChestAngularVelocity.setZero();
      wbmsFinalIKMaxJointDelta = 0.0;
      wbmsFinalIKPreviousValid = false;
    }

    void clearWbmsWalkingPreparationEvents(){
      wbmsWalkingPendingCommandReleaseEvent = false;
      wbmsWalkingApiRejectedNotReadyEvent = false;
      wbmsWalkingApiAcceptedReadyEvent = false;
      wbmsWalkingPreparationStartEvent = false;
      wbmsWalkingPreparationCancelEvent = false;
    }
  };
  DebugData debugData; // デバッグ用のOutPortから出力するためのデータ. AutoStabilizer内の制御処理では使われることは無い. そのため、モード遷移や初期化等の処理にはあまり注意を払わなくて良い

public:
  void clearWbmsPostureCommand(bool resetApplied){
    wbmsRawComVelocityCommand.setZero();
    wbmsRawTorsoAngularVelocityCommand.setZero();
    wbmsVelocityCommandAge = wbmsVelocityCommandTimeout + 1.0;
    wbmsVelocityCommandValid = false;
    if(resetApplied){
      wbmsAppliedComVelocityCommand.setZero();
      wbmsAppliedTorsoAngularVelocityCommand.setZero();
    }
  }

  void clearWbmsPostureReference(){
    wbmsPostureReferenceQ.clear();
    wbmsPostureReferenceJointMask.clear();
    wbmsProjectedChestR.setIdentity();
    wbmsProjectedRobotCom.setZero();
    wbmsRealizedComVelocity.setZero();
    wbmsRealizedTorsoAngularVelocity.setZero();
    wbmsPostureReferenceValid = false;
    wbmsProjectionStatus = WBMS_PROJECTION_NOT_RUN;
    wbmsProjectionAllConstraintsSatisfied = false;
    wbmsProjectionCandidateSafe = false;
    wbmsProjectionSupportHullValid = false;
    wbmsProjectionRootTranslationStep = 0.0;
    wbmsProjectionRootRotationStep = 0.0;
    wbmsProjectionMaxJointStep = 0.0;
    wbmsProjectionMinJointLimitMargin = std::numeric_limits<double>::max();
    wbmsProjectionMaxFootPositionError = 0.0;
    wbmsProjectionMaxFootRotationError = 0.0;
  }

  void resetWbmsPostureControl(){
    clearWbmsPostureCommand(true);
    clearWbmsPostureReference();
    wbmsPostureBaselineValid = false;
    wbmsNominalGenCogBeforeWbmsIntegration.setZero();
    wbmsNominalGenCogBeforeWbmsIntegrationValid = false;
    wbmsWalkingStabilityModeValue = 0.0;
    wbmsOperationModeValue = 0.0;
    clearWbmsWalkingPreparation();
  }

  void clearWbmsWalkingPreparation(){
    isWbmsWalkingStartDelay = false;
    wbmsWalkingStartDelayRemainTime = 0.0;
    wbmsWalkingPreparationPhase = WBMS_WALKING_PREPARATION_INACTIVE;
    wbmsWalkingPreparationFailureCode = WBMS_WALKING_PREPARATION_FAILURE_NONE;
    wbmsWalkingPreparationReleaseRequested = false;
    wbmsWalkingPreparationSnapshotValid = false;
    wbmsWalkingComHeightHoldValid = false;
    wbmsWalkingPreparationElapsedTime = 0.0;
    wbmsWalkingPreparationPhaseElapsedTime = 0.0;
    wbmsWalkingPreparationSettleElapsedTime = 0.0;
    wbmsWalkingPreparationReturnAlpha = 0.0;
    wbmsWalkingPreparationHandoffAlpha = 0.0;
    heldRobotComHeightInFootMid = 0.0;
    wbmsWalkingPreparationStartChestRInFootMid.setIdentity();
    wbmsWalkingPreparationStartRobotComInFootMid.setZero();
    wbmsWalkingPreparationNominalRobotComInFootMid.setZero();
    wbmsWalkingPreparationStartRootR.setIdentity();
    wbmsWalkingPreparationTargetRootR.setIdentity();
    wbmsWalkingPreparationTargetChestRInFootMid.setIdentity();
    wbmsWalkingPreparationTargetRobotComInFootMid.setZero();
    wbmsWalkingPreparationReturnTorsoAngularVelocity.setZero();
    wbmsWalkingPreparationReturnRootAngularVelocity.setZero();
    wbmsWalkingPreparationReturnComVelocity.setZero();
    wbmsWalkingPreparationChestError = 0.0;
    wbmsWalkingPreparationComXYError = 0.0;
    wbmsWalkingPreparationComZError = 0.0;
    wbmsWalkingPreparationRootError = 0.0;
  }

  bool isStatic() const{ // 現在static状態かどうか
    return this->footstepNodesList.size() == 1 && this->footstepNodesList[0].remainTime == 0.0;
  }

  bool shouldKeepFootStepsStaticForWbmsWalkingPreparation() const{
    return
      this->wbmsWalkingPreparationPhase == WBMS_WALKING_PREPARATION_REQUESTED ||
      this->wbmsWalkingPreparationPhase == WBMS_WALKING_PREPARATION_DECELERATING ||
      this->wbmsWalkingPreparationPhase == WBMS_WALKING_PREPARATION_RETURNING ||
      this->wbmsWalkingPreparationPhase == WBMS_WALKING_PREPARATION_HANDOFF ||
      this->wbmsWalkingPreparationPhase == WBMS_WALKING_PREPARATION_READY;
  }

public:
  void init(const cnoid::BodyPtr& robot){
    maxTorque.resize(robot->numJoints(), std::numeric_limits<double>::max());
    jointLimitTables.resize(robot->numJoints());
    jointControllable.resize(robot->numJoints(), true);
    refRobotRaw = robot->clone();
    refRobotRaw->calcForwardKinematics(); refRobotRaw->calcCenterOfMass();
    actRobotRaw = robot->clone();
    actRobotRaw->calcForwardKinematics(); actRobotRaw->calcCenterOfMass();
    refRobot = robot->clone();
    refRobot->calcForwardKinematics(); refRobot->calcCenterOfMass();
    actRobot = robot->clone();
    actRobot->calcForwardKinematics(); actRobot->calcCenterOfMass();
    stServoPGainPercentage.resize(robot->numJoints(), cpp_filters::TwoPointInterpolator<double>(100.0, 0.0, 0.0, cpp_filters::LINEAR));
    stServoDGainPercentage.resize(robot->numJoints(), cpp_filters::TwoPointInterpolator<double>(100.0, 0.0, 0.0, cpp_filters::LINEAR));
    actRobotTqc = robot->clone();
    actRobotTqc->calcForwardKinematics(); actRobotTqc->calcCenterOfMass();
    genRobot = robot->clone();
    genRobot->calcForwardKinematics(); genRobot->calcCenterOfMass();
    debugData.wbmsFinalIKPreviousJointQ.resize(robot->numJoints(), 0.0);
    debugData.resetWbmsFinalIKDiagnostics();
  }

  void push_backEE(const std::string& name_, const std::string& parentLink_, const cnoid::Isometry3& localT_){
    eeName.push_back(name_);
    eeParentLink.push_back(parentLink_);
    eeLocalT.push_back(localT_);
    refEEWrenchOrigin.push_back(cnoid::Vector6::Zero());
    refEEPoseRaw.push_back(cpp_filters::TwoPointInterpolatorSE3(cnoid::Isometry3::Identity(), cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    refEEPose.push_back(cnoid::Isometry3::Identity());
    refEEWrench.push_back(cnoid::Vector6::Zero());
    humanToRobotRatio.push_back(1.0);
    wbmsOffsetPoseMaster.push_back(cnoid::Isometry3::Identity());
    wbmsOffsetPoseSlave.push_back(cnoid::Isometry3::Identity());
    actEEPose.push_back(cnoid::Isometry3::Identity());
    actEEWrench.push_back(cnoid::Vector6::Zero());
    icEEOffset.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    icEETargetPose.push_back(cnoid::Isometry3::Identity());
    abcEETargetPose.push_back(cnoid::Isometry3::Identity());
    stEETargetWrench.push_back(cnoid::Vector6::Zero());
  }

  // startAutoStabilizer時に呼ばれる
  void reset(){
    for(int i=0;i<NUM_LEGS;i++){
      copOffset[i].reset(copOffset[i].getGoal());
      defaultTranslatePos[i].reset(defaultTranslatePos[i].getGoal());
      isManualControlMode[i].reset(isManualControlMode[i].getGoal());
    }

    // 現在の支持脚からの..という性質のportなので、リセットする必要がある
    steppableRegion.clear();
    steppableHeight.clear();
    relLandingHeight = -1e15;
    relLandingNormal = cnoid::Vector3::UnitZ();
    resetWbmsPostureControl();
    debugData.resetWbmsFinalIKDiagnostics();
  }

  // 毎周期呼ばれる. 内部の補間器をdtだけ進める
  void update(double dt){
    for(int i=0;i<NUM_LEGS;i++){
      copOffset[i].interpolate(dt);
      defaultTranslatePos[i].interpolate(dt);
      isManualControlMode[i].interpolate(dt);
    }
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // for debug
  mutable struct timeval prevTime;
  void resetTime() const { gettimeofday(&prevTime, NULL);}
  void printTime(const std::string& message="") const {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    std::cerr << message << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << std::endl;
  }
};

// for debug

inline std::ostream &operator<<(std::ostream &os, const std::vector<GaitParam::FootStepNodes>& footstepNodesList) {
  for(int i=0;i<footstepNodesList.size();i++){
    os << "footstep" << i << std::endl;
    os << " RLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[RLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[RLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << footstepNodesList[i].stepHeight[RLEG][0] << " " << footstepNodesList[i].stepHeight[RLEG][1] << std::endl;
    os << "  goaloffset = " << footstepNodesList[i].goalOffset[RLEG] << std::endl;
    os << " LLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[LLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[LLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << footstepNodesList[i].stepHeight[LLEG][0] << " " << footstepNodesList[i].stepHeight[LLEG][1] << std::endl;
    os << "  goaloffset = " << footstepNodesList[i].goalOffset[LLEG] << std::endl;
    os << " time = " << footstepNodesList[i].remainTime << "[s]" << std::endl;;
  }
  return os;
};

inline std::ostream &operator<<(std::ostream &os, const GaitParam& gaitParam) {
  os << "current" << std::endl;
  os << " RLEG: " << std::endl;
  os << "  pos: " << (gaitParam.genCoords[RLEG].value().translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
  os << "  rot: " << (gaitParam.genCoords[RLEG].value().linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
  os << " LLEG: " << std::endl;
  os << "  pos: " << (gaitParam.genCoords[LLEG].value().translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
  os << "  rot: " << (gaitParam.genCoords[LLEG].value().linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
  os << gaitParam.footstepNodesList << std::endl;
  return os;
};

inline std::ostream &operator<<(std::ostream &os, const std::vector<cnoid::Vector3>& polygon){
  for(int j=0;j<polygon.size();j++){
    os << polygon[j].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]"));
  }
  return os;
}

#endif
