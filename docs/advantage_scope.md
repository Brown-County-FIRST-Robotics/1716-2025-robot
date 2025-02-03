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

## Interfaces in Java

An important concept to understand before we begin is [interfaces](https://www.w3schools.com/java/java_interface.asp) (similar to [traits](https://doc.rust-lang.org/book/ch10-02-traits.html) in Rust). These are constructs in Java used for inheritance where the parent is not (and cannot be) used directly. They are useful for cases where we have various types of something, but we want the same code to apply to all of them. This is done by declaring several methods and variables in the interface that are available to all classes implementing it. These methods must provide a definition, providing the functionality for when the method is called. Each class implementing the interface can have different definitions and functionality for the methods while still being able to be called by the same code, provided it is written to take the interface. Additionally, interfaces can provide default functionality for methods, which means that classes implementing it are no longer required to provide their own definition.

This construct is used heavily in the [WhoAmI](#whoami) system. In this case, each subsystem's IOLayer is an interface, allowing for multiple different versions of that subsystem such as one with REV hardware, one with CTRe hardware, and a simulated version. This is what allows the modularity of this system.

## Adding an `IOLayer` to a Subsystem

This guide will build off of the Command Based Document [subsystem guide](./command_based.md#subsystem).

## Misc. Logging

In order to maintain complete simulation capability using AdvantageScope, we must log a variety of things. These include... This can be done by...
