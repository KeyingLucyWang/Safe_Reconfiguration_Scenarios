#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains the result gatherer and write for CARLA scenarios.
It shall be used from the ScenarioManager only.
"""

import logging
import time


class ResultOutputProvider(object):

    """
    This module contains the _result gatherer and write for CARLA scenarios.
    It shall be used from the ScenarioManager only.
    """

    def __init__(self, data, result, stdout=True, filename=None, junit=None):
        """
        Setup all parameters
        - _data contains all scenario-related information
        - _result is overall pass/fail info
        - _stdout (True/False) is used to (de)activate terminal output
        - _filename is used to (de)activate file output in tabular form
        - _junit is used to (de)activate file output in _junit form
        """
        self._data = data
        self._result = result
        self._stdout = stdout
        self._filename = filename
        self._junit = junit

        self._start_time = time.strftime('%Y-%m-%d %H:%M:%S',
                                         time.localtime(self._data.start_system_time))
        self._end_time = time.strftime('%Y-%m-%d %H:%M:%S',
                                       time.localtime(self._data.end_system_time))

        # logging.basicConfig(filename='test_log.txt',
        #                     filemode='a',
        #                     format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
        #                     datefmt='%H:%M:%S',
        #                     level=logging.INFO)        
        # hdlr = logging.FileHandler('test.log')
        # formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        # hdlr.setFormatter(formatter)
        self.logger = logging.getLogger("ResultProvider")
        self.logger.setLevel(logging.INFO)
        # # self.logger.addHandler(hdlr) 

        self.logger.propagate = False

    def write(self, log_message):
        """
        Public write function
        """
        # if self._stdout:
        #     channel = logging.StreamHandler()
        #     self.logger.addHandler(channel)
        # if self._filename is not None:
        #     filehandle = logging.FileHandler(self._filename)
        #     self.logger.addHandler(filehandle)
        # if self._junit is not None:
        #     self._write_to_junit()
        filehandle = logging.FileHandler("test.log")
        self.logger.addHandler(filehandle)

        if self._stdout or (self._filename is not None):
            # self.logger.info("\n\n")
            # self.logger.info(log_message)
            self._write_to_logger()
            self.logger.info("\n")
            # self.logger.info(
            #     "<-----  Test Configurations  ----->\n"
            # )
            # self.logger.info(log_message)
            # self.logger.info(
            #     "   Test Number |   Scenario One    |   Scenario Two    |   Scenario Three  |   Num. of Reconfigs   "
            # )

            self.logger.handlers = []

    def _write_to_logger(self):
        """
        Writing to logger automatically writes to all handlers in parallel,
        i.e. stdout and file are both captured with this function
        """
        # f = open("test.log", "a")
        # f.write("\n##########################################################")
        # f.write("\nTotal Number of Mode Transitions: {}".format(num_mode_switch))
        self.logger.info("\n##########################################################\n")
        self.logger.info("Test Result: %s", self._result)
        # self.logger.info("Scenario: %s --- Result: %s",
        #                  self._data.scenario_tree.name, self._result)
        # self.logger.info("Start time: %s", (self._start_time))
        # self.logger.info("End time: %s", (self._end_time))
        self.logger.info("Duration: %5.2fs",
                         self._data.scenario_duration_game)
        # self.logger.info("Duration: System Time %5.2fs --- Game Time %5.2fs",
        #                  self._data.scenario_duration_system,
        #                  self._data.scenario_duration_game)
        for ego_vehicle in self._data.ego_vehicles:
            self.logger.info("\nEgo vehicle:  %s", ego_vehicle)

        actor_string = ""
        for actor in self._data.other_actors:
            actor_string += "{}; ".format(actor.type_id)
        self.logger.info("Other actors: %s", actor_string)
        self.logger.info("\n**********************************************************")
        self.logger.info("\n")
        # pylint: disable=line-too-long
        self.logger.info(
            "                Actor             |            Criterion           |   Result    | Actual Value | Expected Value ")
        self.logger.info(
            "-----------------------------------------------------------------------------------------------------------------")
        # pylint: enable=line-too-long
        
        collision = self._data.scenario.get_criteria()[0]
        collision_res = "FAILURE" if collision.test_status == "RUNNING" else collision.test_status

        destination = self._data.scenario.get_criteria()[1]
        destination_res = "SUCCESS" if destination.test_status == "RUNNING" else destination.test_status

        timeout_res = "SUCCESS" if self._data.scenario_duration_game < self._data.scenario.timeout else "FAILURE"
        
        collision_one = self._data.scenario.get_criteria()[2]
        one_res = "FAILURE" if collision_one.test_status == "RUNNING" else collision_one.test_status

        collision_two = self._data.scenario.get_criteria()[3]
        two_res = "FAILURE" if collision_two.test_status == "RUNNING" else collision_two.test_status

        collision_three = self._data.scenario.get_criteria()[4]
        three_res = "FAILURE" if collision_three.test_status == "RUNNING" else collision_three.test_status

        if (one_res == "FAILURE" or two_res == "FAILURE" or three_res == "FAILURE") and collision_res != "FAILURE":
            valid = "INVALID"
        valid = "VALID"

        test_res = "{},{},{},{},{},{}".format(self._result, collision_res, 
                                            destination_res, timeout_res, 
                                            self._data.scenario_duration_game,
                                            valid)

        f = open("test_log.txt", 'a')
        f.write(test_res)
        f.close()

        for criterion in self._data.scenario.get_criteria():
            name_string = criterion.name
            if criterion.optional:
                name_string += " (Opt.)"
            else:
                name_string += " (Req.)"

            self.logger.info("%24s (id=%3d) | %30s | %11s | %12.2f | %12.2f ",
                             criterion.actor.type_id[8:],
                             criterion.actor.id,
                             name_string,
                             # pylint: disable=line-too-long
                             "FAILURE" if criterion.test_status == "RUNNING" else criterion.test_status,
                             # pylint: enable=line-too-long
                             criterion.actual_value,
                             criterion.expected_value_success)

        # Handle timeout separately
        # pylint: disable=line-too-long
        self.logger.info(" %33s | %30s | %11s | %12.2f | %12.2f ",
                         "",
                         "Duration ",
                         "SUCCESS" if self._data.scenario_duration_game < self._data.scenario.timeout else "FAILURE",
                         self._data.scenario_duration_game,
                         self._data.scenario.timeout)
        # pylint: enable=line-too-long

        
        # self.logger.info("\n")

    def _write_to_junit(self):
        """
        Writing to Junit XML
        """
        test_count = 0
        failure_count = 0
        for criterion in self._data.scenario.get_criteria():
            test_count += 1
            if criterion.test_status != "SUCCESS":
                failure_count += 1

        # handle timeout
        test_count += 1
        if self._data.scenario_duration_game >= self._data.scenario.timeout:
            failure_count += 1

        junit_file = open(self._junit, "w")

        junit_file.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")

        test_suites_string = ("<testsuites tests=\"%d\" failures=\"%d\" disabled=\"0\" "
                              "errors=\"0\" timestamp=\"%s\" time=\"%5.2f\" "
                              "name=\"Simulation\" package=\"Scenarios\">\n" %
                              (test_count,
                               failure_count,
                               self._start_time,
                               self._data.scenario_duration_system))
        junit_file.write(test_suites_string)

        test_suite_string = ("  <testsuite name=\"%s\" tests=\"%d\" failures=\"%d\" "
                             "disabled=\"0\" errors=\"0\" time=\"%5.2f\">\n" %
                             (self._data.scenario_tree.name,
                              test_count,
                              failure_count,
                              self._data.scenario_duration_system))
        junit_file.write(test_suite_string)

        for criterion in self._data.scenario.get_criteria():
            testcase_name = criterion.name + "_" + \
                criterion.actor.type_id[8:] + "_" + str(criterion.actor.id)
            result_string = ("    <testcase name=\"{}\" status=\"run\" "
                             "time=\"0\" classname=\"Scenarios.{}\">\n".format(
                                 testcase_name, self._data.scenario_tree.name))
            if criterion.test_status != "SUCCESS":
                result_string += "      <failure message=\"{}\"  type=\"\"><!\[CDATA\[\n".format(
                    criterion.name)
                result_string += "  Actual:   {}\n".format(
                    criterion.actual_value)
                result_string += "  Expected: {}\n".format(
                    criterion.expected_value_success)
                result_string += "\n"
                result_string += "  Exact Value: {} = {}\]\]></failure>\n".format(
                    criterion.name, criterion.actual_value)
            else:
                result_string += "  Exact Value: {} = {}\n".format(
                    criterion.name, criterion.actual_value)
            result_string += "    </testcase>\n"
            junit_file.write(result_string)

        # Handle timeout separately
        result_string = ("    <testcase name=\"Duration\" status=\"run\" time=\"{}\" "
                         "classname=\"Scenarios.{}\">\n".format(
                             self._data.scenario_duration_system,
                             self._data.scenario_tree.name))
        if self._data.scenario_duration_game >= self._data.scenario.timeout:
            result_string += "      <failure message=\"{}\"  type=\"\"><!\[CDATA\[\n".format(
                "Duration")
            result_string += "  Actual:   {}\n".format(
                self._data.scenario_duration_game)
            result_string += "  Expected: {}\n".format(
                self._data.scenario.timeout)
            result_string += "\n"
            result_string += "  Exact Value: {} = {}\]\]></failure>\n".format(
                "Duration", self._data.scenario_duration_game)
        else:
            result_string += "  Exact Value: {} = {}\n".format(
                "Duration", self._data.scenario_duration_game)
        result_string += "    </testcase>\n"
        junit_file.write(result_string)

        junit_file.write("  </testsuite>\n")
        junit_file.write("</testsuites>\n")
        junit_file.close()
