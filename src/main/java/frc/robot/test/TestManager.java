// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.awt.Container;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.ScrollPane;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.plaf.DimensionUIResource;
import javax.swing.text.html.HTMLDocument;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.test.TestUtil.InstantTest;
import frc.robot.test.TestUtil.InstantTestMethod;
import frc.robot.utility.Elastic;

/** A system which will run all tests queued to it and display their results. @author H! */
public class TestManager {
    /* TODO:
     * Make the test runner behave nicely if the testing is cut off halfway through
     * Other Utils
     * 
     * Paralalizable (<- spelled wrong):
     */

    public enum TestSuccess {
        NOTRUN,
        FAIL,
        SUCCESS;
    }

    public TestSuccess testSuccessFromBool(Boolean bool) {
        if (bool == true) {
            return TestSuccess.SUCCESS;
        } else if (bool == false) {
            return TestSuccess.FAIL;
        } else {
            return TestSuccess.NOTRUN;
        }
    }

    /**
     * Stores the results of a test, that being whether it succeeded and any other message
     * it may have provided.
     * 
     * @author H!
     */
    public static class TestResults {
        public TestSuccess m_succeessResult;
        public String m_message;

        public TestResults(TestSuccess succeessResult, String message) {
            m_succeessResult = succeessResult;
            m_message = message;
        }

        public TestResults(TestSuccess succeessResult) {
            this(succeessResult, "");
        }
    }

    /**
     * Used for storing the progression of a test.
     * 
     * @author H!
     */
    protected static enum TestState {
        SETUP,
        RUNNING,
        CLOSEDOWN
    }



    public static Map<String, Map<String, TestResults>> results = new HashMap<String, Map<String, TestResults>>();
    protected static Map<Test, TestSuccess> testsRun = new HashMap<Test, TestSuccess>();

    protected static List<TestGroup> groupsToTest = new ArrayList<TestGroup>();
    protected static List<Test> testsToTest = new ArrayList<Test>();

    protected static int testIndex = 0;

    public static boolean testsFinished = false;
    public static boolean testStarted = false;
    private static int initialPauseLength = 5;
    private static int initialPauseTimer = 0;
    protected static boolean testSelectionMade = false;

    protected static int cyclesRun = 0;

    private static final String tabName = "Ammeter";
    private static ArrayList<GenericEntry> groupSelectionEntries = new ArrayList<>();






    /**
     * Used to add a test group to be tested to the queue. This is an external access point.
     * 
     * @param toTest The {@link TestGroup} to test
     * 
     * @author H!
     */
    public static void queueGroupToTest(TestGroup toTest) {
        //System.out.println("test Queued");
        groupsToTest.add(toTest);
    }

    protected static Test[] getTestsFromGroup(TestGroup group) {
        // WARNING: terribly cursed reflection, keep out
        List<Test> annotatedTests = new ArrayList<Test>();
        try {
            for (Method method : group.getClass().getMethods()) {
                if (method.isAnnotationPresent(InstantTestMethod.class)) {
                    InstantTestMethod testAnnotation = method.getAnnotation(InstantTestMethod.class);
                    annotatedTests.add(new InstantTest(
                        () -> {
                            try {
                                method.invoke(group);
                            } catch (InvocationTargetException e) {
                                if (e.getCause() instanceof AssertionError) {
                                    throw (AssertionError) e.getCause();
                                } else {
                                    throw new RuntimeException(e.toString());
                                }
                            } catch (IllegalAccessException | IllegalArgumentException e) {
                                throw new RuntimeException(e.toString());
                            }
                        }, 
                        testAnnotation.name().length() > 0 ? testAnnotation.name() : method.getName()
                    ));
                }
            }
        } catch (IllegalArgumentException | SecurityException e) {
            throw new RuntimeException(e.toString());
        }
        // This part is fine
        return addListToArray(group.getTests(), annotatedTests);
    }

    /**
     * Should be run when test mode is started by {@link Robot#testInit()}. Resets everything and clears the test queue.
     * 
     * @author H!
     */
    public static void init() {
        //System.out.println("init!");
        groupsToTest.clear();
        testsToTest.clear();
        testIndex = 0;
        initialPauseTimer = initialPauseLength;
        testsFinished = false;
        testSelectionMade = false;
        cyclesRun = 0;
        results = new HashMap<String, Map<String, TestResults>>();
        testsRun = new HashMap<Test, TestSuccess>();
    }

    /**
     * Should be run periodically by {@link Robot#testPeriodic()}. Runs queued tests.
     * 
     * @author H!
     */
    public static void periodic() {
        if (initialPauseTimer > 0) {
            initialPauseTimer--;
            return;
        } else if (initialPauseTimer == 0) {
            showTestGroupSelection();
            initialPauseTimer--;
            return;
        }

        if (!testSelectionMade) {
            for (GenericEntry genericEntry : groupSelectionEntries) {
                
                System.out.println(
                    genericEntry.toString() +
                    " : " +
                    genericEntry.getBoolean(false) +
                    " / " +
                    genericEntry.get().getBoolean()
                );
            }

            return;
        }

        /*if (cyclesRun == 0) {
            // DEBUG
            //System.err.println("Tests Started");
        }*/
        ;
        cyclesRun++;
        //System.out.println("#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#");
        String[] testGroupNames = new String[groupsToTest.size()];
        for (int i = 0; i < groupsToTest.size(); i++) {
            testGroupNames[i] = groupsToTest.get(i).getName();
        }

        if (groupsToTest.size() > 0) {
            results.putIfAbsent(groupsToTest.get(0).getName(), new HashMap<String, TestResults>());
            runTests(groupsToTest.get(0));

        } else {
            if (!testsFinished) {
                displayTestResults();;
                testsFinished = true;
            }
        }
    }

    /**
     * Runs one cycle of the current test on the given test group. Should be run periodically.
     * When all tests are done, removes this element from {@link #groupsToTest} and resets {@link #testIndex}
     * 
     * @param testGroup The test group to run the tests of
     * 
     * @author H!
     */
    protected static void runTests(TestGroup testGroup) {
        if (testsToTest.size() == 0) {
            testsToTest = new ArrayList<Test>(Arrays.asList(getTestsFromGroup(testGroup)));
        }

        /*if (testsToTest.get(0).getName() == "Example Dependent Test") {
            System.out.println("it's time");
        }*/

        if (!testStarted) {
            /* DEPENDENCY LOGIC:
             * 
             * All done, all correct         -> Run test
             * All done, not all correct     -> Mark test as not run, and remove it from the queue
             * Not all done, all correct     -> Buffer with dependencies, move test to the back of the queue
             * Not all done, not all correct -> Mark test as not run, and remove it from the queue
             * 
             * A dependency being "correct" means that its result (success/failure) matched the result the main test required of the dependency.
             */
            // Check if dependencies are already done:
            boolean allDependenciesDone = true;
            boolean allDependenciesCorrect = true;
            for (int i = 0; i < testsToTest.get(0).getDependencies().length; i++) {
                Test test = testsToTest.get(0).getDependencies()[i];
                TestSuccess isRun = testsRun.get(test);
                if (isRun == null) {
                    allDependenciesDone = false;
                } else if (isRun == TestSuccess.FAIL && testsToTest.get(0).getDependencySuccessRequirements()[i] == true) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                } else if (isRun == TestSuccess.SUCCESS && testsToTest.get(0).getDependencySuccessRequirements()[i] == false) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                } else if (isRun == TestSuccess.NOTRUN) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                }
            }

            if (!allDependenciesCorrect) {
                results.get(groupsToTest.get(0).getName()).put(testsToTest.get(0).getName(), new TestResults(TestSuccess.NOTRUN, "Dependencies Not Correct"));
                testsRun.put(testsToTest.remove(0), TestSuccess.NOTRUN);
                if (testsToTest.size() == 0) {
                    groupsToTest.remove(0);
                }
                testStarted = false;
                return;
            } else if (!allDependenciesDone) {
                for (Test test : testsToTest.get(0).getDependencies()) {
                    if (!testsRun.containsKey(test) && !testsToTest.contains(test)) {
                        testsToTest.add(1, test);
                    }
                }

                testsToTest.add(testsToTest.remove(0));
                return; // Return to this method next cycle now that the test list has been updated
            }
            testsToTest.get(0).setup();
        }
        runTest(testsToTest.get(0));
    }

    /** Runs all logic that must run when a tests finishes.
     * This invloves managing reseting counters and preparing the next tests.
     */
    public static void onTestDone(Test test) {
        test.closedown();
        testsToTest.remove(0);
        if (testsToTest.size() == 0) {
            groupsToTest.remove(0);
        }
        testStarted = false;
    }

    /**
     * Logic to run one cycle of a test. Should be run periodically to perform the test.
     * When done, increments to the next test.
     * 
     * @param test The test to run
     * 
     * @author H!
     */
    protected static void runTest(Test test) {
        testStarted = true;
        try {
            test.periodic();
            if (test.isDone()) {
                results.get(groupsToTest.get(0).getName()).put(test.getName(), new TestResults(TestSuccess.SUCCESS));
                testsRun.put(test, TestSuccess.SUCCESS);
                onTestDone(test);
            }
        } catch (AssertionError e) {
            //System.out.println("\n\n\n\n\n\nFAILURE\n\n\n\n\n\n\n");
            results.get(groupsToTest.get(0).getName()).put(test.getName(), new TestResults(TestSuccess.FAIL, e.getMessage()));
            testsRun.put(test, TestSuccess.FAIL);
            onTestDone(test);
        }
    }

    /** The format used to generate the HTML group headers in the test results @author H! */
    protected static final String groupFormat = "<li><h2 class='%2$s'>%1$s | %3$s %4$s </h2> <p><em class='success'>%5$d/%7$d/%8$d Succeess</em> | <em class='fail'>%6$d/%7$d/%8$d Fails</em></p><ul></ul></li>";
    /**A utility method for getting the proper HTML to make a subsytem test result wrapper be displayed
     * 
     * @param resultEntry One entry of the map corresponding to the test group to display
     * @return A {@link String} with the HTML in plaintext repersenting the group header
     * 
     * @author H!
     */
    protected static String getGroupHTMLElement(Entry<String, Map<String, TestResults>> resultEntry) {
        int successCount = 0;
        int performedCount = 0;
        int totalCount = resultEntry.getValue().size();

        for (TestResults testResult : resultEntry.getValue().values()) {
            if (testResult.m_succeessResult == TestSuccess.SUCCESS) {
                successCount++;
                performedCount++;
            } else if (testResult.m_succeessResult != TestSuccess.NOTRUN) {
                performedCount++;
            }
        }

        return String.format(
            groupFormat, 
            resultEntry.getKey(), 
            successCount == performedCount ? "success" : "fail",
            successCount == performedCount ? "✔" : "✗",
            performedCount == totalCount ? "" : "*",
            successCount,
            performedCount - successCount,
            performedCount,
            totalCount
        );
    }

    /** The format used to generate the HTML for individual tests in the test results @author H! */
    protected static final String testFormat = "<li><h3 class='%2$s'>%1$s | %3$s</h3>%4$s</li>";
    /**A utility method for getting the proper HTML to make a test result be displayed
     * 
     * @param testEntry One entry of the map corresponding to the test to display
     * @return A {@link String} with the HTML in plaintext repersenting the test result
     * 
     * @author H!
     */
    protected static String getTestHTMLFormat(Entry<String, TestResults> testEntry) {
        String cssClass = "";
        String resultIcon = "";
        if        (testEntry.getValue().m_succeessResult == TestSuccess.SUCCESS) {
            cssClass = "success";
            resultIcon = "✔";
        } else if (testEntry.getValue().m_succeessResult == TestSuccess.FAIL) {
            cssClass = "fail";
            resultIcon = "✗";
        } else if (testEntry.getValue().m_succeessResult == TestSuccess.NOTRUN) {
            cssClass = "notRun";
            resultIcon = "-";
        }

        return String.format(
            testFormat, 
            testEntry.getKey(), 
            cssClass,
            resultIcon,
            testEntry.getValue().m_message.equals("") ? "" : "<p>" + testEntry.getValue().m_message + "</p>"
        );
    }

    /**Displays the latest results of the integrated tests in a Swing dialog
     * @author H!
     */
    public static void displayTestResults() {
        JTextPane textPane = new JTextPane();
        textPane.setContentType("text/html");
        textPane.setText("<!DOCTYPE html><html><head><style>.fail{color:red}.success{color:green}.notRun{color:grey}</style></head><body><h1>Integrated Test Results</h1><ul id=testGroupList></ul></body></html>");
        HTMLDocument doc = (HTMLDocument) textPane.getDocument();

        for (Entry<String, Map<String, TestResults>> groupEntry : results.entrySet()) {
            try {
                doc.insertAfterStart(doc.getElement("testGroupList"), getGroupHTMLElement(groupEntry));
                
                for (Entry<String, TestResults> testResultEntry : groupEntry.getValue().entrySet()) {
                    doc.insertAfterStart(doc.getElement("testGroupList").getElement(0).getElement(2), getTestHTMLFormat(testResultEntry));
                }


            } catch (Exception e) {
                System.err.println("Test result display generation failed:");
                e.printStackTrace();
            }
        }
        
        JScrollPane scrollPane = new JScrollPane(textPane);

        
        scrollPane.setPreferredSize(new DimensionUIResource(
            600, 
            500
        ));
        
        JOptionPane.showMessageDialog(
            new JTextPane(), 
            scrollPane,
            "Integrated Test Results",
            JOptionPane.PLAIN_MESSAGE
        );
    }


    public static void showTestGroupSelection() {
        // Incoming data
        String[] testGroupNames = new String[groupsToTest.size()];
        for (int i = 0; i < groupsToTest.size(); i++) {
            testGroupNames[i] = groupsToTest.get(i).getName();
        }
        
        // Initialize group selector
        var subsystemSelectionList = Shuffleboard.getTab(tabName).getLayout("Subsystem Selection", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label Position", "HIDDEN"))
        ;
        
        groupSelectionEntries = new ArrayList<>();

        for (int i = 0; i < testGroupNames.length; i++) {
            final int thisI = i;

            // Check if the item is already in the list, and if it is, find it.
            Optional<ShuffleboardComponent<?>> preexistingComponent = subsystemSelectionList
                .getComponents()
                .stream()
                .dropWhile((c) -> c.getTitle() != testGroupNames[thisI])
                .findFirst()
            ;
                
            GenericEntry currentEntry;
            if (preexistingComponent.isPresent()) {
                System.out.println("\n\nElement existed already\n\n");
                currentEntry = ((SimpleWidget) preexistingComponent.get()).getEntry();
            } else {
                System.out.println("\n\nElement added\n\n");
                currentEntry = subsystemSelectionList.add(testGroupNames[i], true)
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .getEntry()
                ;
            }

            groupSelectionEntries.add(currentEntry);
            currentEntry.setBoolean(true);
        }

        // Add confirmation button
        InstantCommand confirmGroupSelectionCommand = new InstantCommand(() -> {
            /*var subsystemSelectionListInternal = Shuffleboard.getTab(tabName).getLayout("Subsystem Selection", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4)
                .withProperties(Map.of("Label Position", "HIDDEN"))
            ;
            for (var component : subsystemSelectionList.getComponents()) {
                var entry = ((SimpleWidget) component).getEntry();
                System.out.println(
                    entry.toString() +
                    " : " +
                    entry.getBoolean(false) +
                    " / " +
                    entry.get().getBoolean()
                );
            } */
            int numberRemoved = 0;
            for (int i = 0; i < groupSelectionEntries.size(); i++) {
                if (!groupSelectionEntries.get(i).getBoolean(false)) {
                    groupsToTest.remove(i - numberRemoved);
                    numberRemoved++;
                }
            }
            testSelectionMade = true;
        });

        if (!Shuffleboard.getTab(tabName).getComponents().stream().anyMatch((c) -> c.getTitle() == "Confirm Selection Choices")) {
            Shuffleboard.getTab(tabName).add("Confirm Selection Choices", confirmGroupSelectionCommand)
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(0, 4)
                .withSize(2, 1)
            ;
        }
        

        /*tab.add("Max speed", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();*/

        /*
        // Root component
        JFrame frame = new JFrame();
        frame.setLayout(new GridBagLayout());
        // Constraints to be used to arrange the button and list properly
        GridBagConstraints constraints = new GridBagConstraints();
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        // Holds the checkboxes
        Container groupList = new Container();
        groupList.setLayout(new GridLayout(testGroupNames.length, 1));
        ScrollPane groupListPane = new ScrollPane();
        groupListPane.add(groupList);
        // Add all checkboxes
        for (String name : testGroupNames) {
            JCheckBox checkBox = new JCheckBox(name, true);
            groupList.add(checkBox);
        }

        // Confirm button
        JButton button = new JButton("Start Tests");
        button.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int numberRemoved = 0;
                // System.err.println(groupList.getComponentCount());
                // System.err.println(groupsToTest.size());
                for (int i = 0; i < groupList.getComponentCount(); i++) {
                    if (!((JCheckBox) groupList.getComponent(i)).isSelected()) {
                        groupsToTest.remove(i - numberRemoved);
                        numberRemoved++;
                    }
                }
                testSelectionMade = true;
                frame.dispose();
            }
        });
        
        // Assemble components into frame
        constraints.fill = GridBagConstraints.BOTH;
        constraints.gridx = 0;
        constraints.gridy = 0;
        constraints.weightx = 1;
        constraints.weighty = 9;
        frame.add(groupListPane, constraints);

        constraints.fill = GridBagConstraints.HORIZONTAL;
        constraints.gridx = 0;
        constraints.gridy = 1;
        constraints.weightx = 1;
        constraints.weighty = 1;
        frame.add(button, constraints);

        frame.pack();

        frame.setVisible(true);*/
    }



    private static <T> T[] addListToArray(T[] array, List<T> list) {
        T[] out = Arrays.copyOf(array, array.length + list.size());
        Iterator<T> iterator = list.iterator();
        for (int i = array.length; i < out.length; i++) {
            out[i] = iterator.next();
        }

        return out;
    }
}
