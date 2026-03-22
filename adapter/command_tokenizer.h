#pragma once

#include "global_constants.h"

#include <sstream>
#include <string>
#include <vector>

namespace gnss
{

// ─── parsed_command ────────────────────────────────────────────────────────────
//
// Result of splitting a raw command string by whitespace.
//
//   raw:  "poll_config /etc/default_ubx_configs.ini"
//   name: "poll_config"
//   args: { "/etc/default_ubx_configs.ini" }
//
// The name holds token[0]; args holds tokens[1..MAX_COMMAND_ARGS].
// Any tokens beyond MAX_COMMAND_ARGS are silently discarded.

struct parsed_command
{
    std::string              name; ///< Command identifier (token[0]).
    std::vector<std::string> args; ///< Up to MAX_COMMAND_ARGS argument tokens.
};

// ─── tokenize ─────────────────────────────────────────────────────────────────
//
// Split @p cmd_str by whitespace and populate a parsed_command.
// Tokens beyond MAX_COMMAND_ARGS are silently ignored.
// Returns a parsed_command with an empty name for a blank input string.

inline parsed_command tokenize(const std::string& cmd_str)
{
    parsed_command result;
    std::istringstream iss(cmd_str);
    std::string token;

    if (!(iss >> token))
    {
        return result; // Empty or whitespace-only input.
    }
    result.name = std::move(token);

    while ((result.args.size() < MAX_COMMAND_ARGS) && (iss >> token))
    {
        result.args.push_back(std::move(token));
    }

    return result;
}

} // namespace gnss
