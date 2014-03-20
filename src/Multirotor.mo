package Multirotor
  package Examples
    model RotorTest "Test of a fixed rotor with fixed input"
      extends Modelica.Icons.Example;
      import Modelica.Mechanics.MultiBody.World;
      import Modelica.Electrical.Analog.Sources.ConstantVoltage;
      import Modelica.Electrical.Analog.Basic.Ground;
      import Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet;
      import Modelica.Mechanics.Rotational.Components.LossyGear;
      import Modelica.Mechanics.Rotational.Sensors.*;
      import Multirotor.Basics.Rotor;
      ConstantVoltage constantvoltage1(V = 12) annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Ground ground1 annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-12.5,-12.5},{12.5,12.5}}, rotation = 0)));
      DC_PermanentMagnet dcpm(VaNominal = 12, IaNominal = 1, wNominal = 1100, Ra = 0.3, Jr = 3.5e-05) annotation(Placement(visible = true, transformation(origin = {-55,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
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
    model RotorDinamicsTest "Test of the rotor on/off dinamics"
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
      DC_PermanentMagnet dcpm(VaNominal = 12, IaNominal = 1, wNominal = 1100, Ra = 0.65, Jr = 3.5e-05) annotation(Placement(visible = true, transformation(origin = {-55,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
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
    model ChassisTest "Test of the weight forces on a simple quadrotor chassis"
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
    model Chassis6RotorTest "Test of the weight forces on a  six rotor chassis"
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
    model Chassis3RotorTest "Test of the weight forces on a  three rotor chassis"
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
    model ArmTest "Test of the forces generated by a fixed input on a fixed arm"
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
    model TestQuadrotor "Simple quadrotor lift test"
      extends Modelica.Icons.Example;
      Multirotor.Basics.Quadrotor quadrotor1 annotation(Placement(visible = true, transformation(origin = {40,40}, extent = {{-25,-25},{25,25}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 1100) annotation(Placement(visible = true, transformation(origin = {-60,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(const.y,quadrotor1.u[4]) annotation(Line(points = {{-49,-20},{12.959,-20},{12.959,26.7819},{12.959,26.7819}}));
      connect(const.y,quadrotor1.u[3]) annotation(Line(points = {{-49,-20},{10.3672,-20},{10.3672,25.054},{10.3672,25.054}}));
      connect(const.y,quadrotor1.u[2]) annotation(Line(points = {{-49,-20},{12.095,-20},{12.095,28.0778},{12.095,28.0778}}));
      connect(const.y,quadrotor1.u[1]) annotation(Line(points = {{-49,-20},{12.095,-20},{12.095,28.0778},{12.095,28.0778}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end TestQuadrotor;
    model TestNRotor "Simple N-rotor lift test (with 4 arms)"
      extends Modelica.Icons.Example;
      Multirotor.Basics.NRotor nrotor(N = 4) annotation(Placement(visible = true, transformation(origin = {40,40}, extent = {{-25,-25},{25,25}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 1100) annotation(Placement(visible = true, transformation(origin = {-60,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(const.y,nrotor.u[4]) annotation(Line(points = {{-49,-20},{12.959,-20},{12.959,26.7819},{12.959,26.7819}}));
      connect(const.y,nrotor.u[3]) annotation(Line(points = {{-49,-20},{10.3672,-20},{10.3672,25.054},{10.3672,25.054}}));
      connect(const.y,nrotor.u[2]) annotation(Line(points = {{-49,-20},{12.095,-20},{12.095,28.0778},{12.095,28.0778}}));
      connect(const.y,nrotor.u[1]) annotation(Line(points = {{-49,-20},{12.095,-20},{12.095,28.0778},{12.095,28.0778}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end TestNRotor;
    model ControllerTest "Test of the arm control under sinusoidal input"
      extends Modelica.Icons.Example;
      Multirotor.Basics.Arm arm1 annotation(Placement(visible = true, transformation(origin = {60,-60}, extent = {{-25,-25},{25,25}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0)));
      Multirotor.Basics.Controller controller1(N = 1, K = 0.004, Ti = 0.055, Vmax = 12) annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-25,-25},{25,25}}, rotation = -90)));
      Modelica.Blocks.Sources.Sine sine1(amplitude = 100, freqHz = 1, offset = 900) annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0)));
    equation
      connect(sine1.y,controller1.setPoint[1]) annotation(Line(points = {{-40.75,60},{5.98131,60},{5.98131,28.75},{6.25,28.75}}));
      connect(controller1.bus[1],arm1.bus) annotation(Line(points = {{5,-25},{5,-44.8598},{35,-44.8598},{35,-45}}));
      connect(world.frame_b,arm1.frame_a) annotation(Line(points = {{-42.5,-60},{34.7664,-60},{34.7664,-59.4393},{34.7664,-59.4393}}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end ControllerTest;
  end Examples;
  package Basics
    import SI = Modelica.SIunits;
    model Rotor "Model that represents a simple fixed-pitch rotor"
      import Modelica.Mechanics.Rotational.Interfaces.*;
      import Modelica.Mechanics.MultiBody.Interfaces.*;
      import Modelica.Mechanics.Rotational.Components.Inertia;
      import Modelica.Mechanics.MultiBody.Parts.Body;
      import Modelica.SIunits.Conversions.NonSIunits.Angle_deg;
      parameter Angle_deg alpha = 10 "The angle of the rotor blades";
      parameter Real bd[3] = {9.96e-09,2.46e-10,4.33e-07} "Drag coefficients";
      parameter Real bl = 3.88e-07 "Lift coefficient";
      parameter SI.Inertia Jr = 3.5e-05 "Rotor Inertia";
      Flange_a flange annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Inertia inertia(J = 0) annotation(Placement(visible = true, transformation(origin = {-40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      InternalSupport internalsupport annotation(Placement(visible = true, transformation(origin = {0,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Frame_b frame annotation(Placement(visible = true, transformation(origin = {0,-100}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 90), iconTransformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      SI.Force L[3] "The lift produced by the rotor";
      SI.Torque Tl "The Drag produced by the rotor";
      SI.AngularVelocity w "The angular velocity of the rotor";
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
    model Quadrotor "Model of a standard quadrotor"
      import Modelica.Blocks.Sources.Constant;
      import Modelica.Blocks.Interfaces.RealInput;
      Arm arm_N annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = -90)));
      Chassis chassis annotation(Placement(visible = true, transformation(origin = {0,20}, extent = {{-26.25,-26.25},{26.25,26.25}}, rotation = 0)));
      Arm arm_E annotation(Placement(visible = true, transformation(origin = {60,20}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = 0)));
      Arm arm_S annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = -90)));
      Arm arm_W annotation(Placement(visible = true, transformation(origin = {-60,20}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = 0)));
      Controller controller(K = 0.004, Ti = 0.055, Vmax = 12) annotation(Placement(visible = true, transformation(origin = {-60,-80}, extent = {{-13.75,-13.75},{13.75,13.75}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u[4] annotation(Placement(visible = true, transformation(origin = {-110,-50}, extent = {{-15,-15},{15,15}}, rotation = 0), iconTransformation(origin = {-110,-50}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(u[4],controller.setPoint[4]) annotation(Line(points = {{-110,-50},{-77.7538,-50},{-77.7538,-76.4579},{-77.7538,-76.4579}}));
      connect(u[3],controller.setPoint[3]) annotation(Line(points = {{-110,-50},{-77.3218,-50},{-77.3218,-78.6177},{-77.3218,-78.6177}}));
      connect(u[2],controller.setPoint[2]) annotation(Line(points = {{-110,-50},{-75.59399999999999,-50},{-75.59399999999999,-76.02589999999999},{-75.59399999999999,-76.02589999999999}}));
      connect(u[1],controller.setPoint[1]) annotation(Line(points = {{-110,-50},{-78.1857,-50},{-78.1857,-77.7538},{-78.1857,-77.7538}}));
      connect(controller.bus[4],arm_W.bus) annotation(Line(points = {{-46.25,-77.25},{-28.3537,-77.25},{-28.3537,33.5366},{-37.1951,33.5366},{-37.1951,33.5366}}));
      connect(controller.bus[3],arm_N.bus) annotation(Line(points = {{-46.25,-77.25},{22.8659,-77.25},{22.8659,50.6098},{13.4146,50.6098},{13.4146,57.3171},{13.4146,57.3171}}));
      connect(controller.bus[2],arm_E.bus) annotation(Line(points = {{-46.25,-77.25},{32.622,-77.25},{32.622,34.1463},{36.8902,34.1463},{36.8902,34.1463}}));
      connect(controller.bus[1],arm_S.bus) annotation(Line(points = {{-46.25,-77.25},{14.3293,-77.25},{14.3293,-17.6829},{14.3293,-17.6829}}));
      connect(arm_N.frame_a,chassis.frame_N) annotation(Line(points = {{1.37773e-15,57.5},{1.37773e-15,46.3415},{-0.609756,46.3415},{-0.609756,46.3415}}));
      connect(arm_E.frame_a,chassis.frame_W) annotation(Line(points = {{37.5,20},{26.5244,20},{26.5244,20.4268},{26.5244,20.4268}}));
      connect(arm_S.frame_a,chassis.frame_S) annotation(Line(points = {{-1.37773e-15,-17.5},{-1.37773e-15,-5.79268},{0.304878,-5.79268},{0.304878,-5.79268}}));
      connect(arm_W.frame_a,chassis.frame_E) annotation(Line(points = {{-37.5,20},{-26.2195,20},{-26.2195,21.0366},{-26.2195,21.0366}}));
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1,1})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1,1}), graphics = {Ellipse(fillColor = {3,104,255}, fillPattern = FillPattern.CrossDiag, extent = {{30,30},{-30,-30}}, endAngle = 360),Rectangle(fillColor = {40,40,255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-70,3},{70,-3}}),Rectangle(origin = {0,-0.25}, fillColor = {0,0,255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-3,70},{3,-70}}),Rectangle(origin = {67,10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Text(origin = {-38.5858,34.3105}, lineColor = {0,0,255}, extent = {{-52.9,38.11},{26.5668,-4.48989}}, textString = "%name"),Rectangle(origin = {67,-10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {67,0}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360),Rectangle(origin = {-67,-10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Rectangle(origin = {-67,10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {-67,0}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360),Rectangle(origin = {-10,67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Rectangle(origin = {10,67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {0,67}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360),Rectangle(origin = {-10,-67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Rectangle(origin = {10,-67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {0,-67}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360)}));
    end Quadrotor;
    model Controller "PI controllers for n rotors"
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
      RealInput setPoint[N] annotation(Placement(visible = true, transformation(origin = {-120,20}, extent = {{-15,-15},{15,15}}, rotation = 0), iconTransformation(origin = {-115,25}, extent = {{-15,-15},{15,15}}, rotation = 0)));
    equation
      for i in 1:N loop
      connect(setPoint[i],PID[i].u_s);
      connect(bus[i].sensor,abs[i].u);
      connect(abs[i].y,PID[i].u_m);
      connect(PID[i].y,gain[i].u);
      connect(gain[i].y,bus[i].control);

      end for;
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-0.26,-0.26}, fillColor = {255,255,255}, fillPattern = FillPattern.Backward, extent = {{-99.20999999999999,99.73999999999999},{99.73999999999999,-99.73999999999999}}),Text(origin = {-2.63,6.32}, lineColor = {0,0,255}, extent = {{-74.01000000000001,33.86},{76.45,-30.2}}, textString = "%name")}));
    end Controller;
    model ChassisNRotor "Model for a multirotor chassis with n arms"
      import Modelica.Mechanics.MultiBody.Parts.FixedRotation;
      import Modelica.Mechanics.MultiBody.Parts.Body;
      import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
      parameter SI.Mass mass = 0.78 "Mass of the chassis (central body)";
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
    model Chassis "Quadrotor chassis model"
      import Modelica.Mechanics.MultiBody.Parts.FixedRotation;
      import Modelica.Mechanics.MultiBody.Parts.Body;
      import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
      parameter SI.Mass mass = 0.78 "Mass of the chassis (central body)";
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
    model Arm "Model of a multirotor arm"
      extends PartialArm;
      import Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet;
      parameter SI.Voltage motorVoltage = 12 "Motor nominal voltage";
      parameter SI.Resistance motorResistance = 0.3 "Motor nominal resistence";
      parameter SI.Inductance motorInductance = 0.0015 "Motor nominal inductance";
      parameter SI.AngularVelocity motorVelocity = 1100 "Motor nominal angular velocity";
      parameter SI.Inertia motorInertia = 3.5e-05 "Motor nominal inertia";
      parameter SI.Inertia rotorInertia = 3.5e-05 "Rotor nominal inertia";
      parameter Real liftCoefficient = 3.88e-07 "Lift coefficient";
      parameter Real dragCoefficients[3] = {9.96e-09,2.46e-10,4.33e-07} "Drag coefficients";
      DC_PermanentMagnet dcpm(VaNominal = motorVoltage, IaNominal = 1, wNominal = motorVelocity, useSupport = true, Jr = motorInertia, Ra = motorResistance, La = motorInductance) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-20,-20},{20,20}}, rotation = 0)));
      Rotor rotor(bd = dragCoefficients, bl = liftCoefficient, Jr = rotorInertia) annotation(Placement(visible = true, transformation(origin = {80,0}, extent = {{-15,-15},{15,15}}, rotation = 0)));
    equation
      connect(signalvoltage.n,dcpm.pin_an) annotation(Line(points = {{-50,40},{-52.1053,40},{-52.1053,19.4737},{-52.1053,19.4737}}));
      connect(signalvoltage.p,dcpm.pin_ap) annotation(Line(points = {{-30,40},{-27.3684,40},{-27.3684,18.9474},{-27.3684,18.9474}}));
      connect(mounting1d.flange_b,dcpm.support) annotation(Line(points = {{-10,-40},{-20.6876,-40},{-20.6876,-21.0672},{-20.6876,-21.0672}}));
      connect(bodycylinder.frame_b,rotor.frame) annotation(Line(points = {{-44.375,-60},{80,-60},{80,-15.2632},{80,-15.2632}}));
      connect(idealgear.flange_a,dcpm.flange) annotation(Line(points = {{10,0},{-19.4737,0},{-19.4737,0},{-19.4737,0}}));
      connect(idealgear.flange_b,rotor.flange) annotation(Line(points = {{30,0},{64.7368,0},{64.7368,0},{64.7368,0}}));
    end Arm;
    extends Modelica.Icons.VariantsPackage;
    partial model PartialArm "Partial model for all multirotor arms"
      import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
      import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
      import Modelica.Mechanics.MultiBody.Parts.Mounting1D;
      import Modelica.Mechanics.Rotational.Sensors.SpeedSensor;
      import Modelica.Mechanics.Rotational.Components.IdealGear;
      import Modelica.Electrical.Analog.Interfaces.*;
      import Modelica.Electrical.Analog.Basic.Ground;
      import Modelica.Electrical.Analog.Sources.SignalVoltage;
      parameter SI.Length length = 0.2 "Lenght of the arm";
      parameter SI.Length diameter = 0.01 "Diameter of the arm";
      parameter SI.Mass mass = 0.18 "Mass of the arm";
      parameter Real gearRatio = 1 "Ratio of the gear";
      BodyCylinder bodycylinder(r = {length,0,0}, diameter = diameter, density = 4 * mass / length / diameter ^ 2 / Modelica.Constants.pi) annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-15.625,-15.625},{15.625,15.625}}, rotation = 0)));
      Mounting1D mounting1d(n = {0,1,0}) annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{10,-10},{-10,10}}, rotation = 360)));
      SpeedSensor speedsensor annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100,-60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Ground ground annotation(Placement(visible = true, transformation(origin = {-80,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      SignalVoltage signalvoltage annotation(Placement(visible = true, transformation(origin = {-40,40}, extent = {{10,-10},{-10,10}}, rotation = 0)));
      Bus bus annotation(Placement(visible = true, transformation(origin = {-100,60}, extent = {{-15,-15},{15,15}}, rotation = -90), iconTransformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
      IdealGear idealgear(ratio = 1) annotation(Placement(visible = true, transformation(origin = {20,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    equation
      connect(speedsensor.flange,idealgear.flange_b) annotation(Line(points = {{10,80},{29.6183,80},{29.6183,-0.610687},{29.6183,-0.610687}}));
      connect(bus.control,signalvoltage.v) annotation(Line(points = {{-100,60},{-100,57.3684},{-41.0526,57.3684},{-41.0526,47.8947},{-41.0526,47.8947}}));
      connect(speedsensor.w,bus.sensor) annotation(Line(points = {{-11,80},{-77.36839999999999,80},{-77.36839999999999,61.5789},{-98.9474,61.5789},{-98.9474,61.5789}}));
      connect(signalvoltage.n,ground.p) annotation(Line(points = {{-50,40},{-80,40},{-80,10.5263},{-80,10.5263}}));
      connect(mounting1d.frame_a,bodycylinder.frame_b) annotation(Line(points = {{-2.44929e-15,-50},{0.526316,-50},{0.526316,-58.9474},{-44.7368,-58.9474},{-44.7368,-58.9474}}));
      connect(frame_a,bodycylinder.frame_a) annotation(Line(points = {{-100,-60},{-76.3158,-60},{-76.3158,-58.9474},{-76.3158,-58.9474}}));
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-16.7966,8.98}, fillColor = {0,0,255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-79.68000000000001,1.32},{66.56999999999999,-20.22}}),Rectangle(origin = {60.3372,-36.964}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Rectangle(origin = {60.5234,36.0719}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Ellipse(origin = {47.2654,-71.86620000000001}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-91.5763,32.8989},{-52.6342,-7.10947}}, endAngle = 360),Text(origin = {-8.99,23.63}, lineColor = {0,0,255}, fillColor = {0,0,255}, extent = {{-81.86,24.84},{51.37,-11.43}}, textString = "%name")}));
    end PartialArm;
    model NRotor
      import Modelica.Blocks.Interfaces.RealInput;
      parameter Integer N = 4 "Number of Arms";
      ChassisNRotor chassisnrotor1(N = N) annotation(Placement(visible = true, transformation(origin = {-60,-20}, extent = {{-21,-21},{21,21}}, rotation = 0)));
      Arm arm[N] annotation(Placement(visible = true, transformation(origin = {42,-22}, extent = {{-22,-22},{22,22}}, rotation = 0)));
      Controller controller(N = N, K = 0.004, Ti = 0.055, Vmax = 12) annotation(Placement(visible = true, transformation(origin = {-20,40}, extent = {{-14,-14},{14,14}}, rotation = 0)));
      RealInput u[N] annotation(Placement(visible = true, transformation(origin = {-120,60}, extent = {{-15,-15},{15,15}}, rotation = 0), iconTransformation(origin = {-110,-50}, extent = {{-15,-15},{15,15}}, rotation = 0)));
    equation
      for i in 1:N loop
      connect(chassisnrotor1.frame[i],arm[i].frame_a);
      connect(controller.bus[i],arm[i].bus);
      connect(u[i],controller.setPoint[i]);

      end for;
      annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1,1})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1,1}), graphics = {Ellipse(fillColor = {3,104,255}, fillPattern = FillPattern.CrossDiag, extent = {{30,30},{-30,-30}}, endAngle = 360),Rectangle(fillColor = {40,40,255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-70,3},{70,-3}}),Rectangle(origin = {0,-0.25}, fillColor = {0,0,255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-3,70},{3,-70}}),Rectangle(origin = {67,10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Text(origin = {-38.5858,34.3105}, lineColor = {0,0,255}, extent = {{-52.9,38.11},{26.5668,-4.48989}}, textString = "%name"),Rectangle(origin = {67,-10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {67,0}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360),Rectangle(origin = {-67,-10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Rectangle(origin = {-67,10}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {-67,0}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360),Rectangle(origin = {-10,67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Rectangle(origin = {10,67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {0,67}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360),Rectangle(origin = {-10,-67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Rectangle(origin = {10,-67}, rotation = -90, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-1.5,6},{1.5,-6}}),Ellipse(origin = {0,-67}, rotation = -90, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-6,6},{6,-6}}, endAngle = 360)}));
    end NRotor;
  end Basics;
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {2,2}), graphics = {Ellipse(origin = {-45,45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {45,45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {45,-45}, fillPattern = FillPattern.Solid, extent = {{-48,48},{48,-48}}, endAngle = 360),Ellipse(origin = {-45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-35,35},{35,-35}}, endAngle = 360),Ellipse(origin = {-45,45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Ellipse(origin = {45,45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Ellipse(origin = {45,-45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillPattern = FillPattern.Solid, extent = {{-2,2},{2,-2}}, endAngle = 360),Polygon(origin = {44.8161,-44.8124}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Polygon(origin = {-44.94,-45.2393}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Polygon(origin = {45.7917,44.8827}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Polygon(origin = {-44.5741,44.7607}, fillPattern = FillPattern.Solid, points = {{-8.54026,-1.22547},{-21.3451,16.7623},{-22.8695,25.2989},{-5.49148,12.494},{-0.918309,1.51844},{8.83779,1.51844},{21.6427,-15.8596},{22.8622,-25.3108},{5.78901,-12.2011},{0.910959,-1.53034},{-8.54026,-1.22547}}),Ellipse(origin = {45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Ellipse(origin = {-45,45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Ellipse(origin = {-45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Ellipse(origin = {45,-45}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, extent = {{-1.5,1.5},{1.5,-1.5}}, endAngle = 360),Rectangle(rotation = 45, fillPattern = FillPattern.Solid, extent = {{-20,20},{20,-20}}),Polygon(origin = {-9.454879999999999,-0.584878}, lineColor = {84,84,84}, fillColor = {84,84,84}, fillPattern = FillPattern.Solid, points = {{6.40442,46.3175},{-1.52241,32.9029},{-8.22972,-12.524},{9.14832,-45.4508},{4.88003,-22.2801},{3.35564,29.2443},{6.40442,46.3175}}),Polygon(origin = {9.9122,1.43927}, lineColor = {255,255,255}, fillColor = {255,255,255}, fillPattern = FillPattern.Solid, points = {{-7.47799,-46.8626},{-3.2097,-23.9968},{-4.42921,27.8325},{-8.697509999999999,44.6008},{-0.7706769999999999,33.93},{8.680540000000001,-14.2407},{-7.47799,-46.8626}})}));
end Multirotor;
