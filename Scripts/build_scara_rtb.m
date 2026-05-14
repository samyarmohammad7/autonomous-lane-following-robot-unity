function robot = build_scara_rtb(d1, d2, d3, d4, q3_limit)
%BUILD_SCARA_RTB Create the RTB model for the SCARA arm used in Part 2.

links(1) = Link([0, d1, d3, 0, 0], 'standard');
links(1).qlim = [-pi, pi];

links(2) = Link([0, d2, d4, pi, 0], 'standard');
links(2).qlim = [-pi, pi];

links(3) = Link([0, 0, 0, 0, 1], 'standard');
links(3).qlim = [0, q3_limit];

robot = SerialLink(links, 'name', 'SCARA RRP');
end
