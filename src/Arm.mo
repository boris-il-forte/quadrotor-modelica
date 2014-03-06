model Arm
  import Modelica.Electrical.Machines.BasicMachines.QuasiStationaryDCMachines.DC_PermanentMagnet;
  import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
  import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
  import Modelica.Electrical.Analog.Interfaces.*;
  import Modelica.Mechanics.Rotational.Sensors.AngleSensor;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Mechanics.Rotational.Components.LossyGear;
  parameter Modelica.SIunits.Length length = 0.2;
  parameter Modelica.SIunits.Length diameter = 0.01;
  DC_PermanentMagnet dcpm(VaNominal = 12, IaNominal = 20, wNominal = 700, useSupport = true, Jr = 0.2) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-20,-20},{20,20}}, rotation = 0)));
  BodyCylinder bodycylinder1(r = {length,0,0}, diameter = diameter) annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-15.625,-15.625},{15.625,15.625}}, rotation = 0)));
  Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100,-60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0), iconTransformation(origin = {-100,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  RealOutput position annotation(Placement(visible = true, transformation(origin = {-100,80}, extent = {{10,-10},{-10,10}}, rotation = 360), iconTransformation(origin = {-100,-20}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  LossyGear lossygear1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {20,0}, extent = {{-15,-15},{15,15}}, rotation = 0)));
  Rotor rotor1 annotation(Placement(visible = true, transformation(origin = {80,0}, extent = {{-15,-15},{15,15}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1d1(n = {0,1,0}) annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{10,-10},{-10,10}}, rotation = 360)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedsensor1 annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{10,-10},{-10,10}}, rotation = 0)));
equation
  connect(speedsensor1.flange,lossygear1.flange_b) annotation(Line(points = {{10,80},{35.2632,80},{35.2632,0.526316},{35.2632,0.526316}}));
  connect(speedsensor1.w,position) annotation(Line(points = {{-11,80},{-92.1053,80},{-92.1053,80.52630000000001},{-92.1053,80.52630000000001}}));
  connect(mounting1d1.frame_a,bodycylinder1.frame_b) annotation(Line(points = {{-2.44929e-15,-50},{0.526316,-50},{0.526316,-58.9474},{-44.7368,-58.9474},{-44.7368,-58.9474}}));
  connect(dcpm.support,mounting1d1.flange_b) annotation(Line(points = {{-20,-20},{-20,-20},{-20,-39.4737},{-11.0526,-39.4737},{-11.0526,-39.4737}}));
  connect(bodycylinder1.frame_b,rotor1.frame) annotation(Line(points = {{-44.375,-60},{78.4211,-60},{78.4211,-16.3158},{78.4211,-16.3158}}));
  connect(rotor1.flange,lossygear1.flange_b) annotation(Line(points = {{65,0},{35.7895,0},{35.7895,0.526316},{35.7895,0.526316}}));
  connect(dcpm.flange,lossygear1.flange_a) annotation(Line(points = {{-20,0},{3.65854,0},{3.65854,0},{3.65854,0}}));
  connect(pin_n,dcpm.pin_an) annotation(Line(points = {{-100,20},{-53.1579,20},{-53.1579,20},{-53.1579,20}}));
  connect(pin_p,dcpm.pin_ap) annotation(Line(points = {{-100,60},{-27.8947,60},{-27.8947,22.1053},{-27.8947,22.1053}}));
  connect(frame_a,bodycylinder1.frame_a) annotation(Line(points = {{-100,-60},{-76.3158,-60},{-76.3158,-58.9474},{-76.3158,-58.9474}}));
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-16.55,-51.02}, fillColor = {177,177,177}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-79.68000000000001,1.32},{66.56999999999999,-20.22}}),Rectangle(origin = {32.63,-21.06}, fillColor = {197,197,197}, fillPattern = FillPattern.VerticalCylinder, extent = {{-0.19,-4.2},{8.42,-28.42}}),Rectangle(origin = {8.758290000000001,-18.5429}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Rectangle(origin = {64.73390000000001,-18.6649}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Line(origin = {-27.9175,23.8005}, points = {{-65.9849,35.3459},{64.8077,35.6507},{64.8077,-32.6419},{65.72239999999999,-35.3858}}),Line(origin = {-31.7073,2.89634}, points = {{-63.1098,16.311},{63.1098,16.311},{63.1098,-16.311}}),Ellipse(origin = {37.2654,-18.182}, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-11.05,11.32},{11.05,-11.32}}, endAngle = 360)}));
end Arm;

