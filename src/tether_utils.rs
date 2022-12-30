pub fn parse_agent_id(topic: &str) -> &str {
    let parts: Vec<&str> = topic.split('/').collect();
    parts[1]
}

pub fn build_topic(agent_type: &str, agent_id: &str, plug_name: &str) -> String {
    format!("{}/{}/{}", agent_type, agent_id, plug_name)
}
