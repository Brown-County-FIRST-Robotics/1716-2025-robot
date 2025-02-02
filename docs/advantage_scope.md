# Advantage Scope Guide

## Table of Contents

1. [Overview](#overview)
    1. [AdvantageKit](#advantagekit)
    2. [WhoAmI](#whoami)

> NOTE: This guide is designed to be read after the [Command Based Document](./command_based.md), as that document explains important concepts referenced throughout this file.

## Overview

The [Command Based Document](./command_based.md) explains the basic layout and construction of a default command based project. In that document, we discuss three distinct layers (or abstractions). Our codebase, on the other hand, has significantly modified subsystems to allow for additional functionality. This functionality is essentially described by two systems we are using:

### AdvantageKit

AdvantageKit is a tool and library combination developed by team 6328 that allows for complete replay of testing and matches entirely through onboard robot logging data. To accomplish this, we add an additional layer beneath subsystems called the `IOLayer`. This layer is used to log motor data including setpoint, encoder position, and temperature. For more information on what this system is and why we use it, please refer to [this video](https://www.youtube.com/watch?v=mmNJjKJG8mw).

### WhoAmI

This is a custom system developed by Colin Finn to allow for more modular code. The essential basis of this system is to allow one variable change to determing the complete configuration of the robot, such as whether it has an arm, is using swerve or meccanum, or is in demo mode. To accomplish this, we split the `IOLayer` of each subsystem into multiple files, each of which can have a different configuration. The most common use of this involves the real motors used on the robot and a simulated version.

## Adding an `IOLayer` to a Subsystem

This guide will build off of the Command Based Document [subsystem guide](./command_based.md#subsystem).

## Misc. Logging

In order to maintain complete simulation capability using AdvantageScope, we must log a variety of things. These include... This can be done by...
