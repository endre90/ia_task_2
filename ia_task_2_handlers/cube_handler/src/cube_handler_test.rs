use r2r;
use futures::future;
use futures::stream::StreamExt;
use r2r::tool_handler_msgs::action::HandleItem;

pub struct Item {
    type_: String,
    instance: String,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "item_handler_test", "rita")?;

    let item_handler_action_client =
        node.create_action_client::<HandleItem::Action>("handle_item")?;

    let waiting_for_item_handler_server = node.is_available(&item_handler_action_client)?;

    let handle = std::thread::spawn(move || loop {
        &node.spin_once(std::time::Duration::from_millis(100));
    });

    println!("waiting for item handler action service...");
    waiting_for_item_handler_server.await?;
    println!("item handler action service available.");

    let silver_gun = Item {
        item: "silver_gun".to_string(),
        instance: "silver_gun_instance_03_above".to_string()
    };
      
    for item in vec![silver_gun] { // }, "silver_box"] {
        for command in vec!["pick"] { // }, "place"] {          
            item_handler_test(
                item.item.as_str(),
                item.instance.as_str(),
                command,
                &tool_handler_action_client,
            ).await?;
        }
    }

    handle.join().unwrap();

    Ok(())
}

async fn item_handler_test(
    item: &str,
    instance: &str,
    command: &str,
    item_handler_action_client: &r2r::ActionClient<HandleItem::Action>,
) -> Result<(), Box<dyn std::error::Error>> {
    
    let g = HandleItem::Goal {
        command: command.to_string(),
        item: item.to_string(),
        instance: instance.to_string()
    };

    let (goal, result, feedback) = tool_handler_action_client
        .send_goal_request(g)
        .expect("could not send goal request")
        .await
        .expect("goal rejected by server");

    println!("goal accepted: {}", goal.uuid);
    
    tokio::task::spawn(async move {
        feedback.for_each(|msg| {
            println!("feedback from tool handler: {}", msg.current_step);
            future::ready(())
        })
        .await
    });

    match result.await {
        Ok((status, msg)) => {
            println!("tool handling action succeeded");
            println!("got result {} with msg {:?}", status, msg);
        }
        Err(e) => {
            println!("tool handling action failed: {:?}", e);
        }
    }

    Ok(())
}
