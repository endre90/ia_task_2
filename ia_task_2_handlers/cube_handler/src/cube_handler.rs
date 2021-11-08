use futures::future;
use futures::stream::Stream;
use futures::Future;
use futures::StreamExt;
use r2r::geometry_msgs::msg::Transform;
use r2r::scene_manipulation_msgs::srv::ManipulateScene;
use r2r::tool_handler_msgs::action::HandleItem;
use r2r::ur_script_generator_msgs::srv::GenerateURScript;
use r2r::ur_script_msgs::action::ExecuteScript;
use r2r::{self, ActionServerGoal};
use std::sync::{Arc, Mutex};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "item_handler", "rita")?;

    let server_requests = node.create_action_server::<HandleItem::Action>("handle_item")?;
    let ur_script_client =
        node.create_client::<GenerateURScript::Service>("ur_script_generator")?;
    let sms_client = node.create_client::<ManipulateScene::Service>("manipulate_scene")?;
    let ur_script_driver_action_client =
        node.create_action_client::<ExecuteScript::Action>("ur_script")?;

    let waiting_for_ur_script_server = node.is_available(&ur_script_client)?;
    let waiting_for_sms_server = node.is_available(&sms_client)?;
    let waiting_for_ur_script_driver_action_server =
        node.is_available(&ur_script_driver_action_client)?;

    let have_item = Arc::new(Mutex::new(false));
    let have_item_clone = have_item.clone();
    let have_item_clone_2 = have_item.clone();

    let tool_io_sub = node.subscribe::<r2r::ur_script_msgs::msg::Measured>("measured")?;

    tokio::task::spawn(async move {
        let sub_callback = sub_callback(tool_io_sub, have_item_clone).await;
        match sub_callback {
            Ok(()) => println!("done."),
            Err(e) => println!("error: {}", e),
        };
    });

    tokio::task::spawn(async move {
        let res = tool_handler_server(
            server_requests,
            have_item_clone_2,
            waiting_for_ur_script_server,
            waiting_for_sms_server,
            waiting_for_ur_script_driver_action_server,
            ur_script_client,
            sms_client,
            ur_script_driver_action_client,
        )
        .await;
        match res {
            Ok(()) => println!("done."),
            Err(e) => println!("error: {}", e),
        };
    });

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    handle.join().unwrap();

    Ok(())
}

async fn sub_callback(
    sub: impl Stream<Item = r2r::ur_script_msgs::msg::Measured> + Unpin,
    have_item: Arc<Mutex<bool>>,
) -> Result<(), Box<dyn std::error::Error>> {
    sub.for_each(|msg| {
        *have_item.lock().unwrap() = msg.in8;
        future::ready(())
    })
    .await;

    Ok(())
}

async fn tool_handler_server(
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<HandleItem::Action>> + Unpin,
    have_item: Arc<Mutex<bool>>,
    waiting_for_ur_script_server: impl Future<Output = r2r::Result<()>>,
    waiting_for_sms_server: impl Future<Output = r2r::Result<()>>,
    waiting_for_ur_script_driver_action_server: impl Future<Output = r2r::Result<()>>,
    ur_script_client: r2r::Client<GenerateURScript::Service>,
    sms_client: r2r::Client<ManipulateScene::Service>,
    ur_script_action_client: r2r::ActionClient<ExecuteScript::Action>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("waiting for sms service...");
    waiting_for_sms_server.await?;
    println!("sms service available.");

    println!("waiting for ur script generator service...");
    waiting_for_ur_script_server.await?;
    println!("ur script generator service available.");

    println!("waiting for ur script driver action service...");
    waiting_for_ur_script_driver_action_server.await?;
    println!("ur script driver action service available.");

    loop {
        match requests.next().await {
            Some(req) => {
                println!("Got goal id: {}", req.uuid);
                let (mut g, mut _cancel) = req.accept().expect("could not accept goal request");
                match g.goal.command.as_str() {
                    "pick" => {
                        if g.goal.item.as_str().starts_with("silver_gun") {
                            // "silver_gun" => {
                            // ===============================================================================
                            println!("step 1 (picking): execute the picking ur_script");
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step: "step 1: execute the picking ur_script".into(),
                            });

                            let position = format!("{}", g.goal.item.as_str());
                            let g_clone = g.clone();
                            let script = generate_ur_script(
                                g_clone,
                                &ur_script_client,
                                "pick_svt",
                                "lage",
                                "move_j",
                                "1.37",
                                "[0.001, 0.001, 0.047]",
                                0.1,
                                0.1,
                                &position,
                                "svt_tcp",
                            )
                            .await;

                            let g_clone = g.clone();
                            if !execute_ur_script(g_clone, &script, &ur_script_action_client).await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            // ===============================================================================
                            println!(
                                "step 2 (picking): checking if we have actually picked somehing"
                            );
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step:
                                    "step 2 (picking): checking if we have actually picked somehing"
                                        .into(),
                            });
                            if !*have_item.lock().unwrap() {
                                println!("step x (picking): we failed to pick");
                                let result = HandleItem::Result { success: false };
                                g.succeed(result).expect("could not send result");
                            }
                            // ===============================================================================
                            println!(
                                "step 3 (picking): changing the item parent in the tf tree to svt_tcp"
                            );
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step:
                                    "step 3 (picking): changing the item parent in the tf tree to svt_tcp"
                                        .into(),
                            });

                            let g_clone = g.clone();
                            let mut item_clone = g.goal.item.clone();
                            item_clone.truncate("silver_gun_instance".len() + 3);
                            println!("step t (picking): base {}", g.goal.item.clone());
                            println!("step t (picking): truncated {}", item_clone.clone());
                            if !sms_the_item(g_clone, "svt_tcp", item_clone.as_str(), &sms_client)
                                .await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            // ===============================================================================
                            let result = HandleItem::Result { success: true };
                            g.succeed(result).expect("could not send result");
                        } else if g.goal.item.as_str().starts_with("silver_box") {
                            // ===============================================================================
                            println!("step 1 (picking): execute the picking ur_script");
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step: "step 1: execute the picking ur_script".into(),
                            });

                            let position = format!("{}", g.goal.item.as_str());
                            let g_clone = g.clone();
                            let script = generate_ur_script(
                                g_clone,
                                &ur_script_client,
                                "pick_sponge",
                                "lage",
                                "move_j",
                                "1.83",
                                "[0.0, 0.003, 0.069]",
                                0.1,
                                0.1,
                                &position,
                                "sponge_tcp",
                            )
                            .await;

                            let g_clone = g.clone();
                            if !execute_ur_script(g_clone, &script, &ur_script_action_client).await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            // ===============================================================================
                            println!("step 2 (picking): assuming that we picked something...");
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step:
                                    "step 2 (picking): assuming that we picked something...".into(),
                            });
                            // if !*have_item.lock().unwrap() {
                            //     println!(
                            //         "step x (picking): we failed to pick"
                            //     );
                            //     let result = HandleItem::Result {success: false};
                            //     g.succeed(result).expect("could not send result");
                            // }
                            // ===============================================================================
                            println!(
                                "step 3 (picking): changing the item parent in the tf tree to sponge_tcp"
                            );
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step:
                                    "step 3 (picking): changing the item parent in the tf tree to sponge_tcp"
                                        .into(),
                            });

                            let g_clone = g.clone();
                            let mut item_clone = g.goal.item.clone();
                            item_clone.truncate("silver_box_instance".len() + 3);
                            println!("step t (picking): base {}", g.goal.item.clone());
                            println!("step t (picking): truncated {}", item_clone.clone());
                            if !sms_the_item(
                                g_clone,
                                "sponge_tcp",
                                item_clone.as_str(),
                                &sms_client,
                            )
                            .await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            // ===============================================================================
                            let result = HandleItem::Result { success: true };
                            g.succeed(result).expect("could not send result");
                        } else {
                            println!("unknown item to pick {}", g.goal.item.clone(),);
                            continue;
                            // panic!("unknown item to pick")
                        }
                        // _ => panic!("unknown item to pick"),
                    }
                    "place" => {
                        if g.goal.item.as_str().starts_with("silver_gun") {
                            // ===============================================================================
                            println!("step 1 (placing): execute the droping ur_script");
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step: "step 1: execute the droping ur_script".into(),
                            });

                            let position = format!("{}", g.goal.item.as_str());
                            let g_clone = g.clone();
                            let script = generate_ur_script(
                                g_clone,
                                &ur_script_client,
                                "drop_vacuum",
                                "lage",
                                "move_j",
                                "1.37",
                                "[0.001, 0.001, 0.047]",
                                0.1,
                                0.1,
                                &position,
                                "svt_tcp",
                            )
                            .await;

                            let g_clone = g.clone();
                            if !execute_ur_script(g_clone, &script, &ur_script_action_client).await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            // ===============================================================================
                            println!(
                                "step 2 (placing): changing the item parent in the tf tree to agv pos"
                            );
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step:
                                    "step 2 (placing): changing the item parent in the tf tree to agv pos"
                                        .into(),
                            });

                            let mut item_clone = g.goal.item.clone();
                            item_clone.truncate("silver_gun_instance".len() + 3);

                            let position_clone = g.goal.type_.clone().split_off(6);
                            println!(
                                "step t (placing) {} to {}",
                                item_clone.clone(),
                                position_clone.clone()
                            );

                            let g_clone = g.clone();
                            if !sms_the_item(
                                g_clone,
                                position_clone.as_str(),
                                item_clone.as_str(),
                                &sms_client,
                            )
                            .await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            println!(
                                "step t (placing): placed {} to {}",
                                g.goal.item.clone(),
                                position.clone()
                            );
                            // ===============================================================================
                            let result = HandleItem::Result { success: true };
                            g.succeed(result).expect("could not send result");
                        } else if g.goal.item.as_str().starts_with("silver_box") {
                            // ===============================================================================
                            println!("step 1 (placing): execute the droping ur_script");
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                current_step: "step 1: execute the droping ur_script".into(),
                            });

                            let position = format!("{}", g.goal.type_.as_str());
                            let g_clone = g.clone();
                            let script = generate_ur_script(
                                g_clone,
                                &ur_script_client,
                                "drop_vacuum",
                                "lage",
                                "move_j",
                                "1.83",
                                "[0.0, 0.003, 0.069]",
                                0.1,
                                0.1,
                                &position,
                                "sponge_tcp",
                            )
                            .await;

                            let g_clone = g.clone();
                            if !execute_ur_script(g_clone, &script, &ur_script_action_client).await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            // ===============================================================================
                            println!(
                                    "step 2 (placing): changing the item parent in the tf tree to agv pos"
                                );
                            let _ = g.publish_feedback(HandleItem::Feedback {
                                    current_step:
                                        "step 2 (placing): changing the item parent in the tf tree to agv pos"
                                            .into(),
                                });

                            let mut item_clone = g.goal.item.clone();
                            item_clone.truncate("silver_box_instance".len() + 3);

                            let position_clone = g.goal.type_.clone().split_off(6);
                            println!(
                                "step t (placing) {} to {}",
                                item_clone.clone(),
                                position_clone.clone()
                            );

                            let g_clone = g.clone();
                            if !sms_the_item(
                                g_clone,
                                position_clone.as_str(),
                                item_clone.as_str(),
                                &sms_client,
                            )
                            .await
                            {
                                let _ = g.abort(HandleItem::Result::default());
                                continue;
                            }
                            println!(
                                "step t (placing): placed {} to {}",
                                g.goal.item.clone(),
                                position.clone()
                            );
                            // ===============================================================================
                            let result = HandleItem::Result { success: true };
                            g.succeed(result).expect("could not send result");
                        } else {
                            println!("unknown item to pick {}", g.goal.item.clone(),);
                            continue;
                        }
                    }
                    _ => {
                        println!("unknown command {}", g.goal.command.clone(),);
                        continue;
                    }
                }
            }
            None => (),
        }
    }
}

async fn sms_the_item(
    g: ActionServerGoal<HandleItem::Action>,
    parent: &str,
    item: &str,
    sms_client: &r2r::Client<ManipulateScene::Service>,
) -> bool {
    println!("sending request to sms");
    let _ = g.publish_feedback(HandleItem::Feedback {
        current_step: "sending request to sms".into(),
    });

    let sms_request = ManipulateScene::Request {
        remove: false,
        child_frame: item.to_string(),
        parent_frame: parent.to_string(),
        transform: Transform::default(),
        same_position_in_world: false,
    };

    let sms_response = sms_client
        .request(&sms_request)
        .expect("could not send sms request")
        .await
        .expect("cancelled");

    println!("request to sms sent");
    let _ = g.publish_feedback(HandleItem::Feedback {
        current_step: "request to sms sent".into(),
    });

    sms_response.result
}

async fn generate_ur_script(
    g: ActionServerGoal<HandleItem::Action>,
    ur_script_client: &r2r::Client<GenerateURScript::Service>,
    command: &str,
    robot_id: &str,
    move_type: &str,
    payload_mass: &str,
    payload_cog: &str,
    velocity: f64,
    acceleration: f64,
    position: &str,
    tcp_name: &str,
) -> String {
    let mut default_req = GenerateURScript::Request::default();
    default_req.command = command.to_string();

    println!("sending request to ur script generator");
    let _ = g.publish_feedback(HandleItem::Feedback {
        current_step: "sending request to ur script generator".into(),
    });

    let ursg_req = GenerateURScript::Request {
        command: command.to_string(),
        robot_id: robot_id.to_string(),
        move_type: move_type.to_string(),
        payload_mass: payload_mass.to_string(),
        payload_cog: payload_cog.to_string(),
        acceleration: acceleration,
        velocity: velocity,
        goal_feature_name: position.to_string(),
        tcp_name: tcp_name.to_string(),
    };

    let ursg_response = ur_script_client
        .request(&ursg_req)
        .expect("could not send ur script generator request")
        .await
        .expect("cancelled");

    println!("generated ur script");
    let _ = g.publish_feedback(HandleItem::Feedback {
        current_step: "generated ur script".into(),
    });

    return ursg_response.ur_script;
}

async fn execute_ur_script(
    g: ActionServerGoal<HandleItem::Action>,
    script: &str,
    ur_script_driver_action_client: &r2r::ActionClient<ExecuteScript::Action>,
) -> bool {
    let goal = ExecuteScript::Goal {
        script: script.to_string(),
    };

    println!("sending request to ur script driver");
    let _ = g.publish_feedback(HandleItem::Feedback {
        current_step: "sending request to ur script driver".into(),
    });

    let (_goal, result, _feedback) = ur_script_driver_action_client
        .send_goal_request(goal)
        .expect("could not send goal request")
        .await
        .expect("did not get goal");

    match result.await {
        Ok((status, msg)) => {
            println!("got result {} with msg {:?}", status, msg);
            println!("executing the ur script succeeded");
            let _ = g.publish_feedback(HandleItem::Feedback {
                current_step: "executing the ur script succeeded".into(),
            });
            return true;
        }
        Err(e) => {
            println!("action failed: {:?}", e);
            println!("executing the ur script failed, aborting tool handle goal");
            let _ = g.publish_feedback(HandleItem::Feedback {
                current_step: "executing the ur script failed, aborting tool handle goal".into(),
            });
            return false;
        }
    }
}
