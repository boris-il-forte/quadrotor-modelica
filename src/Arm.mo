model Arm
  parameter SI.Length length;
  Modelica.Mechanics.Rotational.Components.Gearbox gearbox1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {40,0}, extent = {{-20,-20},{20,20}}, rotation = 0)));
  Modelica.Electrical.Machines.BasicMachines.QuasiStationaryDCMachines.DC_PermanentMagnet dcpm(VaNominal = 5, IaNominal = 0.25, wNominal = 314) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-20,-20},{20,20}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder1 annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-15.625,-15.625},{15.625,15.625}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100,-60}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 0), iconTransformation(origin = {-100,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Rotor rotor1 annotation(Placement(visible = true, transformation(origin = {77.5,-57.5}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = -90)));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(pin_n,dcpm.pin_an) annotation(Line(points = {{-100,20},{-53.1579,20},{-53.1579,20},{-53.1579,20}}));
  connect(pin_p,dcpm.pin_ap) annotation(Line(points = {{-100,60},{-27.8947,60},{-27.8947,22.1053},{-27.8947,22.1053}}));
  connect(gearbox1.flange_b,rotor1.flange_a) annotation(Line(points = {{60,0},{77.36839999999999,0},{77.5,-40},{77.5,-40}}));
  connect(bodycylinder1.frame_b,rotor1.frame_a) annotation(Line(points = {{-44.375,-60},{59.4737,-60},{60,-57.5},{60,-57.5}}));
  connect(frame_a,bodycylinder1.frame_a) annotation(Line(points = {{-100,-60},{-76.3158,-60},{-76.3158,-58.9474},{-76.3158,-58.9474}}));
  connect(bodycylinder1.frame_b,dcpm.support) annotation(Line(points = {{-44.375,-60},{-20.1869,-60},{-20.1869,-21.6822},{-20,-21.6822},{-20,-20}}));
  connect(dcpm.support,gearbox1.support) annotation(Line(points = {{-20,-20},{40,-20},{40,-20},{40,-20}}));
  connect(dcpm.flange,gearbox1.flange_a) annotation(Line(points = {{-20,0},{19.8131,0},{20,0},{20,0}}));
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-12.8947,-58.9474}, extent = {{-83.9474,20.5263},{83.9474,-20.5263}}),Rectangle(origin = {32.6316,-9.47368}, extent = {{-8.421049999999999,28.4211},{8.421049999999999,-28.4211}}),Ellipse(origin = {32.6316,32.3684}, extent = {{-11.0526,11.3158},{11.0526,-11.3158}}, endAngle = 360),Rectangle(origin = {0.526316,32.3684}, extent = {{-18.9474,5},{18.9474,-5}}),Rectangle(origin = {64.2105,31.5789}, extent = {{-18.9474,5.26316},{18.9474,-5.26316}})}));
end Arm;

