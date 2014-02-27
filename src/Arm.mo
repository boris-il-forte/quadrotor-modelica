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
  DC_PermanentMagnet dcpm(VaNominal = 5, IaNominal = 0.25, wNominal = 314) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-20,-20},{20,20}}, rotation = 0)));
  BodyCylinder bodycylinder1(r = {length,0,0}, diameter = diameter) annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-15.625,-15.625},{15.625,15.625}}, rotation = 0)));
  Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100,-60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0), iconTransformation(origin = {-100,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  AngleSensor anglesensor1 annotation(Placement(visible = true, transformation(origin = {-40,80}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  RealOutput position annotation(Placement(visible = true, transformation(origin = {-100,80}, extent = {{10,-10},{-10,10}}, rotation = 360), iconTransformation(origin = {-100,-20}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  LossyGear lossygear1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {20,0}, extent = {{-15,-15},{15,15}}, rotation = 0)));
equation
  connect(lossygear1.flange_b,anglesensor1.flange) annotation(Line(points = {{35,0},{35.3659,0},{35.3659,80.1829},{-30.1829,80.1829},{-30.1829,80.1829}}));
  connect(lossygear1.support,dcpm.support) annotation(Line(points = {{20,-15},{19.2073,-15},{19.2073,-20.122},{-21.9512,-20.122},{-21.9512,-18.5976},{-21.9512,-18.5976}}));
  connect(dcpm.flange,lossygear1.flange_a) annotation(Line(points = {{-20,0},{3.65854,0},{3.65854,0},{3.65854,0}}));
  connect(anglesensor1.phi,position) annotation(Line(points = {{-51,80},{-93.5976,80},{-93.5976,79.5732},{-93.5976,79.5732}}));
  connect(pin_n,dcpm.pin_an) annotation(Line(points = {{-100,20},{-53.1579,20},{-53.1579,20},{-53.1579,20}}));
  connect(pin_p,dcpm.pin_ap) annotation(Line(points = {{-100,60},{-27.8947,60},{-27.8947,22.1053},{-27.8947,22.1053}}));
  connect(frame_a,bodycylinder1.frame_a) annotation(Line(points = {{-100,-60},{-76.3158,-60},{-76.3158,-58.9474},{-76.3158,-58.9474}}));
  connect(bodycylinder1.frame_b,dcpm.support) annotation(Line(points = {{-44.375,-60},{-20.1869,-60},{-20.1869,-21.6822},{-20,-21.6822},{-20,-20}}));
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-16.55,-51.02}, fillColor = {177,177,177}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-79.68000000000001,1.32},{66.56999999999999,-20.22}}),Rectangle(origin = {32.63,-21.06}, fillColor = {197,197,197}, fillPattern = FillPattern.VerticalCylinder, extent = {{-0.19,-4.2},{8.42,-28.42}}),Rectangle(origin = {8.758290000000001,-18.5429}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Rectangle(origin = {64.73390000000001,-18.6649}, fillColor = {247,247,247}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-18.95,5},{18.95,-5}}),Line(origin = {-27.9175,23.8005}, points = {{-65.9849,35.3459},{64.8077,35.6507},{64.8077,-32.6419},{65.72239999999999,-35.3858}}),Line(origin = {-31.7073,2.89634}, points = {{-63.1098,16.311},{63.1098,16.311},{63.1098,-16.311}}),Ellipse(origin = {37.2654,-18.182}, fillColor = {71,71,71}, fillPattern = FillPattern.Sphere, extent = {{-11.05,11.32},{11.05,-11.32}}, endAngle = 360)}));
end Arm;

