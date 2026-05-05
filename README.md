# 2.12/2.120 Challenge 2: Mobile Robot Step-by-Step Sequence

1. Identify Dinner Table April tag.
2.Initialize a concurrent background process (using your depth
or RGB camera) to constantly scan for the moving obstacle during
all forward/reverse translation movements. Program the robot to
brake/stop when the obstacle is detected and resume once clear,
allowing you to hit the 3-point maximum for obstacle
avoidance.**
3.navigate to beginning of ramp slope A
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
14. Continue forward until just beside preset tray.
15. Make a 90-degree right turn.
16. Scan for the undertable AprilTag.
17. Get the chassis into position to grip the tray.
18. Grab the tray.
19. Reverse a bit.
20. Turn right 90 degrees.
21. Identify the AprilTag on the other side of the ramp (Base of Slope B).
22. Path plan to navigate back to the base of Slope B.
23. **[Loaded Slope B - Point 1]** Navigate UP Slope B using IMU data to keep the tray level, then reverse DOWN Slope B to the base of Slope B.
24. **[Loaded Slope B - Point 2]** Navigate UP Slope B again, then reverse DOWN Slope B.
25. **[Loaded Slope B - Point 3]** Navigate UP Slope B a third time, then reverse DOWN Slope B. (Max 3 points secured for Slope B).
27. Navigate toward the dinner table (around ramp or on ramp
    whichever is more reliable).
28. When close enough, scan for the dinner table AprilTag.
29. Approach further, making informed adjustments.
30. Deposit the food tray.
31. Navigate right to where the dishwasher tray is.
32. When facing table again, use AprilTag and adjust positioning.
33. Grab the dishwasher tray.
34. Navigate to the dishwasher.
35. When close enough, identify the AprilTags and adjust positioning.
36. Deposit the dishwasher tray on the dishwasher.
