#include "PCH.h"
#include "Hooks.h"


// R5.8: Forma Motus owns only non-human procedural locomotion pose warping.

namespace logger = SKSE::log;

namespace
{
    void SetupLog()
    {
        auto logsFolder = SKSE::log::log_directory();
        if (!logsFolder) {
            return;
        }

        auto pluginName = SKSE::PluginDeclaration::GetSingleton()->GetName();
        auto logFilePath = *logsFolder / std::format("{}.log", pluginName);

        auto fileLoggerPtr = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFilePath.string(), true);
        auto loggerPtr = std::make_shared<spdlog::logger>("log", std::move(fileLoggerPtr));

        spdlog::set_default_logger(std::move(loggerPtr));
        spdlog::set_level(spdlog::level::info);
        spdlog::flush_on(spdlog::level::warn);
    }

    void MessageHandler(SKSE::MessagingInterface::Message* a_msg)
    {
        switch (a_msg->type) {
        case SKSE::MessagingInterface::kDataLoaded:
            Hooks::Install();
            break;

        case SKSE::MessagingInterface::kPostLoadGame:
        case SKSE::MessagingInterface::kNewGame:
            Hooks::ResetRuntimeState();
            break;
        }
    }
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
    SKSE::Init(a_skse);
    SetupLog();

    if (auto* messaging = SKSE::GetMessagingInterface(); messaging) {
        messaging->RegisterListener(MessageHandler);
    } else {
        logger::critical("Failed to obtain SKSE messaging interface.");
        return false;
    }

    logger::info(
        "Forma Motus R5.13 loaded | sourceRev=513 SPIDER_OAR_CHAIN NON_HUMAN_ONLY. Player and ActorTypeNPC races are excluded from every Forma layer. Non-humans retain procedural AME+Bank+Stride+Slope with optional Incessus provider. Core dependencies=Universal Kinematics + Motus Continuus Pose Kinematics + Feet of Skyrim; Terrain SDK is soft. FullBodyIK owns foot lock and final body constraints.");
    return true;
}

