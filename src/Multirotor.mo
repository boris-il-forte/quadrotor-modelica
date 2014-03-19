package Multirotor
  package Examples
    model RotorTest
      extends Modelica.Icons.Example;
      import Modelica.Mechanics.MultiBody.World;
      import Modelica.Electrical.Analog.Sources.ConstantVoltage;
      import Modelica.Electrical.Analog.Basic.Ground;
      import Modelica.Mechanics.Rotational.Components.LossyGear;
      import Modelica.Mechanics.Rotational.Sensors.*;
      import Multirotor.Basics.Rotor;
      ConstantVoltage constantvoltage1 annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Ground ground1 annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-12.5,-12.5},{12.5,12.5}}, rotation = 0)));
      Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(VaNominal = 12, IaNominal = 1, wNominal = 314) annotation(Placement(visible = true, transformation(origin = {-55,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
      LossyGear lossygear1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {5,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
      AngleSensor angle annotation(Placement(visible = true, transformation(origin = {60,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      SpeedSensor angularSpeed annotation(Placement(visible = true, transformation(origin = {60,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Rotor rotor1 annotation(Placement(visible = true, transformation(origin = {40,-60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
    equation
      connect(rotor1.frame,world.frame_b) annotation(Line(points = {{30,-60},{30,-59.4512},{-50.3049,-59.4512},{-50.3049,-59.4512}}));
      connect(rotor1.flange,lossygear1.flange_b) annotation(Line(points = {{40,-50},{40,15.2439},{20.4268,15.2439},{20.4268,15.2439}}));
      connect(lossygear1.flange_b,angle.flange) annotation(Line(points = {{20,15},{20.4268,15},{20.4268,40.2439},{50,40.2439},{50,40.2439}}));
      connect(angularSpeed.flange,lossygear1.flange_b) annotation(Line(points = {{50,0},{20.4268,0},{20.4268,15.2439},{20.4268,15.2439}}));
      connect(dcpm.flange,lossygear1.flange_a) annotation(Line(points = {{-40,15},{-10.6707,15},{-10.6707,14.6341},{-10.6707,14.6341}}));
      connect(dcpm.pin_ap,constantvoltage1.p) annotation(Line(points = {{-46,30},{-39.3293,30},{-39.3293,60.3659},{-50,60.3659},{-50,60}}));
      connect(constantvoltage1.n,dcpm.pin_an) annotation(Line(points = {{-70,60},{-79.5732,60},{-79.5732,30.1829},{-66,30},{-64,30}}));
      connect(ground1.p,constantvoltage1.n) annotation(Line(points = {{-60,92.5},{-70.4268,92.5},{-70.4268,60.6707},{-70.4268,60.6707}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end RotorTest;
    model RotorDinamicsTest
      extends Modelica.Icons.Example;
      import Modelica.Mechanics.MultiBody.World;
      import Modelica.Electrical.Analog.Sources.SignalVoltage;
      import Modelica.Electrical.Analog.Basic.Ground;
      import Modelica.Mechanics.Rotational.Components.LossyGear;
      import Modelica.Mechanics.Rotational.Sensors.*;
      import Multirotor.Basics.Rotor;
      import Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet;
      import Modelica.Blocks.Sources.Pulse;
      SignalVoltage signalvoltage1 annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Ground ground1 annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-12.5,-12.5},{12.5,12.5}}, rotation = 0)));
      DC_PermanentMagnet dcpm(VaNominal = 12, IaNominal = 1, wNominal = 314) annotation(Placement(visible = true, transformation(origin = {-55,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
      LossyGear lossygear1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {5,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
      AngleSensor angle annotation(Placement(visible = true, transformation(origin = {60,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      SpeedSensor angularSpeed annotation(Placement(visible = true, transformation(origin = {60,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Rotor rotor1 annotation(Placement(visible = true, transformation(origin = {40,-60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Pulse pulse1(amplitude = 12, period = 40, nperiod = 1) annotation(Placement(visible = true, transformation(origin = {40,80}, extent = {{10,-10},{-10,10}}, rotation = 0)));
    equation
      connect(signalvoltage1.v,pulse1.y) annotation(Line(points = {{-60,67},{11.5888,67},{11.5888,80},{29.1589,80},{29.1589,80}}));
      connect(rotor1.frame,world.frame_b) annotation(Line(points = {{30,-60},{30,-59.4512},{-50.3049,-59.4512},{-50.3049,-59.4512}}));
      connect(rotor1.flange,lossygear1.flange_b) annotation(Line(points = {{40,-50},{40,15.2439},{20.4268,15.2439},{20.4268,15.2439}}));
      connect(lossygear1.flange_b,angle.flange) annotation(Line(points = {{20,15},{20.4268,15},{20.4268,40.2439},{50,40.2439},{50,40.2439}}));
      connect(angularSpeed.flange,lossygear1.flange_b) annotation(Line(points = {{50,0},{20.4268,0},{20.4268,15.2439},{20.4268,15.2439}}));
      connect(dcpm.flange,lossygear1.flange_a) annotation(Line(points = {{-40,15},{-10.6707,15},{-10.6707,14.6341},{-10.6707,14.6341}}));
      connect(dcpm.pin_ap,signalvoltage1.p) annotation(Line(points = {{-46,30},{-39.3293,30},{-39.3293,60.3659},{-50,60.3659},{-50,60}}));
      connect(signalvoltage1.n,dcpm.pin_an) annotation(Line(points = {{-70,60},{-79.5732,60},{-79.5732,30.1829},{-66,30},{-64,30}}));
      connect(ground1.p,signalvoltage1.n) annotation(Line(points = {{-60,92.5},{-70.4268,92.5},{-70.4268,60.6707},{-70.4268,60.6707}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end RotorDinamicsTest;
    model ChassisTest
      extends Modelica.Icons.Example;
      import Multirotor.Basics.Chassis;
      Chassis chassis1 annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder1(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder2(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,40}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder3(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {40,0}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder4(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{-10,-10},{10,10}}, rotation = 90)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-80,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(world.frame_b,bodycylinder1.frame_a) annotation(Line(points = {{-70,0},{-50.0778,0},{-50.0778,0.311042},{-50.0778,0.311042}}));
      connect(bodycylinder4.frame_b,chassis1.frame_S) annotation(Line(points = {{6.12323e-16,-30},{6.12323e-16,-9.953340000000001},{-0.622084,-9.953340000000001},{-0.622084,-9.953340000000001}}));
      connect(bodycylinder3.frame_b,chassis1.frame_W) annotation(Line(points = {{30,0},{10.2644,0},{10.2644,0},{10,0}}));
      connect(chassis1.frame_N,bodycylinder2.frame_b) annotation(Line(points = {{0,10},{0,10},{0,30.1711},{0,30.1711}}));
      connect(bodycylinder1.frame_b,chassis1.frame_E) annotation(Line(points = {{-30,0},{-10.8865,0},{-10.8865,0},{-10.8865,0}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})));
    end ChassisTest;
    model Chassis6RotorTest
      extends Modelica.Icons.Example;
      import Multirotor.Basics.ChassisNRotor;
      import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-80,-40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      ChassisNRotor chassisnrotor1(N = 6) annotation(Placement(visible = true, transformation(origin = {40,-80}, extent = {{-10,-10},{10,10}}, rotation = 90)));
      BodyCylinder bodycylinder1(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,-40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder2(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder3(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder4(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder5(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {80,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder6(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {80,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(chassisnrotor1.frame[4],bodycylinder4.frame_b) annotation(Line(points = {{40,-70},{40,67.6829},{14.6341,67.6829},{14.6341,79.5732},{-30,79.5732},{-30,80}}));
      connect(chassisnrotor1.frame[3],bodycylinder3.frame_b) annotation(Line(points = {{40,-70},{40,24.3902},{16.4634,24.3902},{16.4634,40.2439},{-30,40.2439},{-30,40}}));
      connect(bodycylinder2.frame_b,chassisnrotor1.frame[2]) annotation(Line(points = {{-30,0},{0.304878,0},{0.304878,-70},{40,-70}}));
      connect(world.frame_b,bodycylinder1.frame_a) annotation(Line(points = {{-70,-40},{-10.061,-40},{-50,-40},{-50,-40}}));
      connect(bodycylinder1.frame_b,chassisnrotor1.frame[1]) annotation(Line(points = {{-30,-40},{-0.304878,-40},{-0.304878,-70},{40,-70}}));
      connect(chassisnrotor1.frame[6],bodycylinder6.frame_b) annotation(Line(points = {{40,-70},{40,30.1829},{96.9512,30.1829},{96.9512,39.3293},{90,39.3293},{90,40}}));
      connect(chassisnrotor1.frame[5],bodycylinder5.frame_b) annotation(Line(points = {{40,-70},{40,68.2927},{95.122,68.2927},{95.122,80.1829},{90,80.1829},{90,80}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})));
    end Chassis6RotorTest;
    model Chassis3RotorTest
      extends Modelica.Icons.Example;
      import Multirotor.Basics.ChassisNRotor;
      import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      ChassisNRotor chassisnrotor1 annotation(Placement(visible = true, transformation(origin = {40,-40}, extent = {{-10,-10},{10,10}}, rotation = 90)));
      BodyCylinder bodycylinder1(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder2(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      BodyCylinder bodycylinder3(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(bodycylinder3.frame_b,chassisnrotor1.frame[3]) annotation(Line(points = {{10,80},{39.939,80},{39.939,-29.878},{39.939,-29.878}}));
      connect(bodycylinder2.frame_b,chassisnrotor1.frame[2]) annotation(Line(points = {{10,40},{40.5488,40},{40.5488,-29.878},{40.5488,-29.878}}));
      connect(bodycylinder1.frame_b,chassisnrotor1.frame[1]) annotation(Line(points = {{10,0},{40.2439,0},{40.2439,-29.878},{40.2439,-29.878}}));
      connect(bodycylinder1.frame_a,world.frame_b) annotation(Line(points = {{-10,0},{-29.5732,0},{-29.5732,0.304878},{-29.5732,0.304878}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})));
    end Chassis3RotorTest;
    model ArmTest
      extends Modelica.Icons.Example;
      import Modelica.Mechanics.MultiBody.World;
      import Modelica.Electrical.Analog.Sources.ConstantVoltage;
      import Modelica.Electrical.Analog.Basic.Ground;
      import Multirotor.Basics.Arm;
      inner Modelica.Mechanics.MultiBody.World world(g = 9.81) annotation(Placement(visible = true, transformation(origin = {-80,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Arm arm1 annotation(Placement(visible = true, transformation(origin = {20,20}, extent = {{-25,-25},{25,25}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 12) annotation(Placement(visible = true, transformation(origin = {-40,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(const.y,arm1.bus.control) annotation(Line(points = {{-29,40},{-11.2805,40},{-11.2805,34.7561},{-6.40244,34.7561},{-6.40244,34.7561}}));
      connect(arm1.frame_a,world.frame_b) annotation(Line(points = {{-5,20},{-70,20},{-70,20.5263},{-70,20.5263}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end ArmTest;
    extends Modelica.Icons.ExamplesPackage;
  end Examples;
  package Basics
    model Rotor
      import Modelica.Mechanics.Rotational.Interfaces.*;
      import Modelica.Mechanics.MultiBody.Interfaces.*;
      import Modelica.Mechanics.Rotational.Components.Inertia;
      import Modelica.Mechanics.MultiBody.Parts.Body;
      import Modelica.SIunits.*;
      import Modelica.SIunits.Conversions.NonSIunits.Angle_deg;
      parameter Angle_deg alpha = 10 "The angle of the rotor blades";
      parameter Real bd[3] = {9.96e-09,2.46e-10,4.33e-07};
      parameter Real bl = 3.88e-07;
      parameter Modelica.SIunits.Inertia Jr = 3.5e-05;
      Flange_a flange annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Inertia inertia(J = 0) annotation(Placement(visible = true, transformation(origin = {-40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      InternalSupport internalsupport annotation(Placement(visible = true, transformation(origin = {0,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Frame_b frame annotation(Placement(visible = true, transformation(origin = {0,-100}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 90), iconTransformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Force L[3] "The lift produced by the rotor";
      Torque Tl "The Drag produced by the rotor";
      AngularVelocity w "The angular velocity of the rotor";
    equation
      w = der(inertia.flange_b.phi);
      L[1] = 0;
      L[2] = alpha * bl * w ^ 2;
      L[3] = 0;
      Tl = sign(w) * (bd[1] * w ^ 2 + bd[2] * w ^ 2 * alpha ^ 2 + bd[3] * abs(w) * alpha);
      internalsupport.tau = Tl;
      L + frame.f = zeros(3);
      frame.t = zeros(3);
      connect(internalsupport.flange,inertia.flange_b) annotation(Line(points = {{0,20},{-29.5931,20},{-29.5931,19.9753},{-29.5931,19.9753}}));
      connect(inertia.flange_a,flange) annotation(Line(points = {{-50,20},{-100.645,20},{-100.645,20.3226},{-100.645,20.3226}}));
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-40.26,0.79}, fillColor = {203,203,203}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-21.32,6.05},{21.32,-6.05}}),Rectangle(origin = {44.5982,1.55449}, fillColor = {203,203,203}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-21.32,6.05},{21.32,-6.05}}),Ellipse(origin = {-9.0989,-11.5018}, fillColor = {184,184,184}, fillPattern = FillPattern.Sphere, extent = {{-12.11,-12.63},{34.79,37.79}}, endAngle = 360),Text(origin = {4.42,58.54}, lineColor = {0,0,255}, extent = {{-89.18000000000001,24.7},{89.18000000000001,-24.7}}, textString = "%name")}));
    end Rotor;
    model Quadrotor
      import Modelica.Blocks.Sources.Constant;
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Controller controller(K = 1, Ti = 0.1, Vmax = 12) annotation(Placement(visible = true, transformation(origin = {-70,-80}, extent = {{-13.75,-13.75},{13.75,13.75}}, rotation = 0)));
      Constant const[4](each k = 11000) annotation(Placement(visible = true, transformation(origin = {-66.25,-33.75}, extent = {{13.75,-13.75},{-13.75,13.75}}, rotation = 0)));
      Arm arm_N annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = -90)));
      Chassis chassis annotation(Placement(visible = true, transformation(origin = {0,20}, extent = {{-26.25,-26.25},{26.25,26.25}}, rotation = 0)));
      Arm arm_E annotation(Placement(visible = true, transformation(origin = {60,20}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = 0)));
      Arm arm_S annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = -90)));
      Arm arm_W annotation(Placement(visible = true, transformation(origin = {-60,20}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = 0)));
    equation
      connect(controller.bus[4],arm_W.bus) annotation(Line(points = {{-56.25,-77.25},{-28.3537,-77.25},{-28.3537,33.5366},{-37.1951,33.5366},{-37.1951,33.5366}}));
      connect(controller.bus[3],arm_N.bus) annotation(Line(points = {{-56.25,-77.25},{22.8659,-77.25},{22.8659,50.6098},{13.4146,50.6098},{13.4146,57.3171},{13.4146,57.3171}}));
      connect(controller.bus[2],arm_E.bus) annotation(Line(points = {{-56.25,-77.25},{32.622,-77.25},{32.622,34.1463},{36.8902,34.1463},{36.8902,34.1463}}));
      connect(controller.bus[1],arm_S.bus) annotation(Line(points = {{-56.25,-77.25},{14.3293,-77.25},{14.3293,-17.6829},{14.3293,-17.6829}}));
      connect(arm_N.frame_a,chassis.frame_N) annotation(Line(points = {{1.37773e-15,57.5},{1.37773e-15,46.3415},{-0.609756,46.3415},{-0.609756,46.3415}}));
      connect(arm_E.frame_a,chassis.frame_W) annotation(Line(points = {{37.5,20},{26.5244,20},{26.5244,20.4268},{26.5244,20.4268}}));
      connect(arm_S.frame_a,chassis.frame_S) annotation(Line(points = {{-1.37773e-15,-17.5},{-1.37773e-15,-5.79268},{0.304878,-5.79268},{0.304878,-5.79268}}));
      connect(arm_W.frame_a,chassis.frame_E) annotation(Line(points = {{-37.5,20},{-26.2195,20},{-26.2195,21.0366},{-26.2195,21.0366}}));
      connect(const.y,controller.setPoint) annotation(Line(points = {{-81.375,-33.75},{-92.98779999999999,-33.75},{-92.98779999999999,-76.2195},{-87.5,-76.2195},{-87.5,-76.2195}}));
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1,1})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1,1})));
    end Quadrotor;
    model Controller
      import Modelica.Blocks.Continuous.LimPID;
      import Modelica.Blocks.Interfaces.RealInput;
      import Modelica.Blocks.Math.Gain;
      import Modelica.Blocks.Math.Abs;
      parameter Integer N = 4 "Number of controllers";
      parameter Real K "Gain of controllers";
      parameter Real Ti "Time constat of integrator blocks";
      parameter Real Vmax "Maximum voltage";
      Gain gain[N](k = array((-1) ^ i for i in 1:N)) annotation(Placement(visible = true, transformation(origin = {40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      LimPID PID[N](each controllerType = Modelica.Blocks.Types.SimpleController.PI, each yMin = 0, each k = K, each Ti = Ti, each yMax = Vmax) annotation(Placement(visible = true, transformation(origin = {-20,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Abs abs[N] annotation(Placement(visible = true, transformation(origin = {40,-20}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Bus bus[N] annotation(Placement(visible = true, transformation(origin = {100,20}, extent = {{-15,-15},{15,15}}, rotation = 90), iconTransformation(origin = {100,20}, extent = {{-10,-10},{10,10}}, rotation = 90)));
      RealInput setPoint[N] annotation(Placement(visible = true, transformation(origin = {-115,25}, extent = {{-15,-15},{15,15}}, rotation = 0), iconTransformation(origin = {-115,25}, extent = {{-15,-15},{15,15}}, rotation = 0)));
    equation
      for i in 1:N loop
      connect(setPoint[i],PID[i].u_s);
      connect(bus[i].sensor,abs[i].u);
      connect(abs[i].u,PID[i].u_m);
      connect(PID[i].y,gain[i].u);
      connect(gain[i].y,bus[i].control);

      end for;
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-0.26,-0.26}, fillColor = {255,255,255}, fillPattern = FillPattern.Backward, extent = {{-99.20999999999999,99.73999999999999},{99.73999999999999,-99.73999999999999}}),Text(origin = {-2.63,6.32}, lineColor = {0,0,255}, extent = {{-74.01000000000001,33.86},{76.45,-30.2}}, textString = "%name")}));
    end Controller;
    model ChassisNRotor
      import Modelica.SIunits.Mass;
      import Modelica.Mechanics.MultiBody.Parts.FixedRotation;
      import Modelica.Mechanics.MultiBody.Parts.Body;
      import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
      parameter Mass mass = 0.78 "Mass of the chassis (central body)";
      parameter Integer N = 3 "Number of Arms";
      Frame_a frame[N] annotation(Placement(visible = true, transformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      FixedRotation fixedrotation[N - 1](n = array({0,1,0} for i in 1:N - 1), angle = array(360 * i / N for i in 1:N - 1));
      Body body annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{10,-10},{-10,10}}, rotation = 0)));
    equation
      connect(body.frame_a,frame[1]) annotation(Line(points = {{10,0},{99.6951,0},{99.6951,0},{99.6951,0}}));
      for i in 1:N - 1 loop
      connect(body.frame_a,fixedrotation[i].frame_a);
      connect(fixedrotation[i].frame_b,frame[i + 1]);

      end for;
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0,0})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0,0}), graphics = {Ellipse(fillColor = {3,104,255}, fillPattern = FillPattern.CrossDiag, extent = {{100,100},{-100,-100}}, endAngle = 360),Text(origin = {-0.15,109.61}, lineColor = {0,0,255}, extent = {{-55.34,23.93},{55.34,-23.93}}, textString = "%name")}));
    end ChassisNRotor;
    model Chassis
      import Modelica.SIunits.Mass;
      import Modelica.Mechanics.MultiBody.Parts.FixedRotation;
      import Modelica.Mechanics.MultiBody.Parts.Body;
      import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
      parameter Mass mass = 0.78 "Mass of the chassis (central body)";
      FixedRotation fixedrotation2(n = {0,1,0}, angle = 270) annotation(Placement(visible = true, transformation(origin = {40,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      FixedRotation fixedrotation1(n = {0,1,0}, angle = 180) annotation(Placement(visible = true, transformation(origin = {40,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      FixedRotation fixedrotation3(n = {0,1,0}, angle = 90) annotation(Placement(visible = true, transformation(origin = {40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Body body(m = mass) annotation(Placement(visible = true, transformation(origin = {-40,-60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Frame_a frame_N annotation(Placement(visible = true, transformation(origin = {100,-40}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {0,100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Frame_a frame_E annotation(Placement(visible = true, transformation(origin = {100,80}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Frame_a frame_S annotation(Placement(visible = true, transformation(origin = {100,40}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Frame_a frame_W annotation(Placement(visible = true, transformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(body.frame_a,fixedrotation2.frame_a) annotation(Line(points = {{-40,-50},{-40,80.1829},{29.2683,80.1829},{29.2683,80.1829}}));
      connect(body.frame_a,fixedrotation1.frame_a) annotation(Line(points = {{-40,-50},{-40,40.8537},{30.1829,40.8537},{30.1829,40.8537}}));
      connect(body.frame_a,fixedrotation3.frame_a) annotation(Line(points = {{-40,-50},{-40,0.304878},{29.878,0.304878},{29.878,0.304878}}));
      connect(frame_N,body.frame_a) annotation(Line(points = {{100,-40},{-40.2439,-40},{-40.2439,-49.6951},{-40.2439,-49.6951}}));
      connect(fixedrotation2.frame_b,frame_E) annotation(Line(points = {{50,80},{100.305,80},{100.305,80.48779999999999},{100.305,80.48779999999999}}));
      connect(fixedrotation1.frame_b,frame_S) annotation(Line(points = {{50,40},{99.6951,40},{99.6951,39.939},{99.6951,39.939}}));
      connect(fixedrotation3.frame_b,frame_W) annotation(Line(points = {{50,0},{99.6951,0},{99.6951,0.609756},{99.6951,0.609756}}));
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Ellipse(origin = {-0.15,-0.46}, fillColor = {3,104,255}, fillPattern = FillPattern.CrossDiag, extent = {{99.54000000000001,99.54000000000001},{-99.54000000000001,-99.54000000000001}}, endAngle = 360),Rectangle(origin = {0.15,0}, fillColor = {40,40,255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-97.09999999999999,10.06},{97.09999999999999,-10.06}}),Rectangle(origin = {1.07,2.74057}, fillColor = {0,0,255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-10.1511,96.15940000000001},{8.69,-100.3}}),Text(origin = {113.572,75.001}, lineColor = {0,0,255}, extent = {{-52.9,38.11},{69.97,-18.3}}, textString = "%name")}));
    end Chassis;
    expandable connector Bus "Bus to connect the controller to the multirotor arm"
      extends Modelica.Icons.SignalSubBus;
      Real sensor "measured angular speed";
      Real control "control signal (voltage)";
    end Bus;
    model Arm
      import Modelica.Electrical.Machines.BasicMachines.QuasiStationaryDCMachines.DC_PermanentMagnet;
      import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
      import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
      import Modelica.Electrical.Analog.Interfaces.*;
      import Modelica.Mechanics.Rotational.Sensors.AngleSensor;
      import Modelica.Blocks.Interfaces.RealOutput;
      import Modelica.Mechanics.Rotational.Components.LossyGear;
      import Modelica.SIunits.*;
      parameter Length length = 0.2 "Lenght of the arm";
      parameter Length diameter = 0.01 "Diameter of the arm";
      parameter Mass mass = 0.18 "Mass of the arm";
      parameter Voltage motorVoltage = 12 "Motor nominal voltage";
      parameter Current motorCurrent = 1 "Motor nominal current";
      parameter AngularVelocity motorVelocity = 1100 "Motor nominal angular velocity";
      parameter Inertia motorInertia = 3.5e-05 "Motor nominal inertia";
      parameter Inertia rotorInertia = 3.5e-05 "Rotor nominal inertia";
      parameter Real liftCoefficient = 3.88e-07 "Lift coefficient";
      parameter Real dragCoefficients[3] = {9.96e-09,2.46e-10,4.33e-07} "Drag coefficients";
      DC_PermanentMagnet dcpm(VaNominal = motorVoltage, IaNominal = motorCurrent, wNominal = motorVelocity, useSupport = true, Jr = motorInertia) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-20,-20},{20,20}}, rotation = 0)));
      BodyCylinder bodycylinder1(r = {length,0,0}, diameter = diameter, density = 4 * mass / length / diameter ^ 2 / Modelica.Constants.pi) annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-15.625,-15.625},{15.625,15.625}}, rotation = 0)));
      Rotor rotor1(bd = dragCoefficients, bl = liftCoefficient, Jr = rotorInertia) annotation(Placement(visible = true, transformation(origin = {80,0}, extent = {{-15,-15},{15,15}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1d1(n = {0,1,0}) annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{10,-10},{-10,10}}, rotation = 360)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedsensor1 annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100,-60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-80,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalvoltage1 annotation(Placement(visible = true, transformation(origin = {-40,40}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Multirotor.Basics.Bus bus annotation(Placement(visible = true, transformation(origin = {-100,60}, extent = {{-15,-15},{15,15}}, rotation = -90), iconTransformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      Modelica.Mechanics.Rotational.Components.IdealGear idealgear1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {20,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(speedsensor1.flange,idealgear1.flange_b) annotation(Line(points = {{10,80},{29.6183,80},{29.6183,-0.610687},{29.6183,-0.610687}}));
      connect(idealgear1.flange_a,dcpm.flange) annotation(Line(points = {{10,0},{-19.2366,0},{-19.2366,0.305344},{-19.2366,0.305344}}));
      connect(idealgear1.flange_b,rotor1.flange) annotation(Line(points = {{30,0},{66.2595,0},{66.2595,0.305344},{66.2595,0.305344}}));
      connect(bus.control,signalvoltage1.v) annotation(Line(points = {{-100,60},{-100,57.3684},{-41.0526,57.3684},{-41.0526,47.8947},{-41.0526,47.8947}}));
      connect(speedsensor1.w,bus.sensor) annotation(Line(points = {{-11,80},{-77.36839999999999,80},{-77.36839999999999,61.5789},{-98.9474,61.5789},{-98.9474,61.5789}}));
      connect(signalvoltage1.n,ground1.p) annotation(Line(points = {{-50,40},{-80,40},{-80,10.5263},{-80,10.5263}}));
      connect(signalvoltage1.p,dcpm.pin_ap) annotation(Line(points = {{-30,40},{-26.8421,40},{-26.8421,20},{-26.8421,20}}));
      connect(signalvoltage1.n,dcpm.pin_an) annotation(Line(points = {{-50,40},{-53.1579,40},{-53.1579,20.5263},{-53.1579,20.5263}}));
      connect(mounting1d1.frame_a,bodycylinder1.frame_b) annotation(Line(points = {{-2.44929e-15,-50},{0.526316,-50},{0.526316,-58.9474},{-44.7368,-58.9474},{-44.7368,-58.9474}}));
      connect(dcpm.support,mounting1d1.flange_b) annotation(Line(points = {{-20,-20},{-20,-20},{-20,-39.4737},{-11.0526,-39.4737},{-11.0526,-39.4737}}));
      connect(bodycylinder1.frame_b,rotor1.frame) annotation(Line(points = {{-44.375,-60},{78.4211,-60},{78.4211,-16.3158},{78.4211,-16.3158}}));
      connect(frame_a,bodycylinder1.frame_a) annotation(Line(points = {{-100,-60},{-76.3158,-60},{-76.3158,-58.9474},{-76.3158,-58.9474}}));
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-16.55,8.98}, fillColor = {0,0,255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-79.68000000000001,1.32},{66.56999999999999,-20.22}}),Rectangle(origin = {60.3372,-36.964}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Rectangle(origin = {60.5234,36.0719}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Ellipse(origin = {47.2654,-71.86620000000001}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-91.5763,32.8989},{-52.6342,-7.10947}}, endAngle = 360),Text(origin = {-8.99,23.63}, lineColor = {0,0,255}, fillColor = {0,0,255}, extent = {{-81.86,24.84},{51.37,-11.43}}, textString = "%name")}));
    end Arm;
    extends Modelica.Icons.VariantsPackage;
  end Basics;
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {2,2}), graphics = {Ellipse(origin = {-45,45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {45,45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {45,-45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {-45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {-45,45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Ellipse(origin = {45,45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Ellipse(origin = {45,-45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Polygon(origin = {44.8161,-44.8124}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Polygon(origin = {-44.94,-45.2393}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Polygon(origin = {45.7917,44.8827}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Polygon(origin = {-44.5741,44.7607}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Ellipse(origin = {45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Ellipse(origin = {-45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Ellipse(origin = {45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Rectangle(rotation = 45, fillPattern = FillPattern.Solid, extent = {{-20,20},{20,-20}}),Polygon(origin = {-9.454879999999999,-0.584878}, lineColor = {84,84,84}, fillColor = {84,84,84}, fillPattern = FillPattern.Solid, points = {{6.40442,46.3175},{-1.52241,32.9029},{-8.22972,-12.524},{9.14832,-45.4508},{4.88003,-22.2801},{3.35564,29.2443},{6.40442,46.3175}}),Polygon(origin = {9.9122,1.43927}, lineColor = {255,255,255}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, points = {{-7.47799,-46.8626},{-3.2097,-23.9968},{-4.42921,27.8325},{-8.697509999999999,44.6008},{-0.7706769999999999,33.93},{8.680540000000001,-14.2407},{-7.47799,-46.8626}})}));
end Multirotor;