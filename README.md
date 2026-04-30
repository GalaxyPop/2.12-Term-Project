# 2.12/2.120 Challenge 2: Mobile Robot Step-by-Step Sequence

1. Identify AprilTag to go up the ramp.
2. Align with the AprilTag and navigate forward until right at the beginning of the ramp (Base of Slope A).
3. **Initialize a concurrent background process (using your depth or RGB camera) to constantly scan for the moving obstacle during all forward/reverse translation movements. Program the robot to brake/stop when the obstacle is detected and resume once clear, allowing you to hit the 3-point maximum for obstacle avoidance.**
4. **[Empty Traversal 1]** Navigate up Slope A.
5. Make a 90-degree right turn at the peak.
6. Navigate down Slope B to complete the first full empty traversal.
7. **[Empty Traversal 2]** Reverse back up Slope B.
8. Make a 90-degree left turn (in reverse) at the peak.
9. Reverse down Slope A to complete the second full empty traversal.
10. **[Empty Traversal 3]** Navigate forward up Slope A again.
11. Make a 90-degree right turn at the peak.
12. Navigate down Slope B to complete the third full empty traversal, securing the maximum 3 points for this category.
13. Move forward a bit until the preset tray AprilTag is seen.
14. Continue forward to the set position.
15. Make a 90-degree right turn.
16. Scan for the AprilTag for the food tray prep area.
17. Get the chassis into position to grip the tray.
18. Grab the tray.
19. Reverse a bit.
20. Turn right 90 degrees.
21. Identify the AprilTag on the other side of the ramp (Base of Slope A).
22. Path plan to navigate back to the base of Slope A.
23. **[Loaded Slope A - Point 1]** Navigate UP Slope A using IMU data to keep the tray level, then reverse DOWN Slope A to the starting position.
24. **[Loaded Slope A - Point 2]** Navigate UP Slope A again, then reverse DOWN Slope A.
25. **[Loaded Slope A - Point 3]** Navigate UP Slope A a third time, then reverse DOWN Slope A. (Max 3 points secured for Slope A).
26. Navigate UP Slope A to cross the peak, make a 90-degree right turn, and navigate DOWN Slope B.
27. **[Loaded Slope B - Point 1]** Reverse UP Slope B, then navigate DOWN Slope B.
28. **[Loaded Slope B - Point 2]** Reverse UP Slope B, then navigate DOWN Slope B.
29. **[Loaded Slope B - Point 3]** Reverse UP Slope B, then navigate DOWN Slope B. (Max 3 points secured for Slope B).
30. Navigate toward the dinner table.
31. When close enough, scan for the dinner table AprilTag.
32. Approach further, making informed adjustments.
33. Deposit the food tray.
34. Navigate to where the dishwasher tray is.
35. When close enough, identify the AprilTag and adjust positioning.
36. Grab the dishwasher tray.
37. Navigate to the dishwasher.
38. When close enough, identify the AprilTag and adjust positioning.
39. Deposit the dishwasher tray on the dishwasher.
