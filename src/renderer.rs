use std::path::Path;

use crate::{RendererResult, RobotBuilder, RobotFile};
use anyhow::Result;

pub trait Renderer {
    // fn start(&mut self) {} // 打开窗口/连接/记录流
    // fn shutdown(&mut self) {} // 收尾

    fn set_additional_search_path(&mut self, path: impl AsRef<Path>) -> RendererResult<&mut Self>;
}

pub trait RendererRobot {
    type RR<R>;
    type RB<'a, R: RobotFile>: RobotBuilder<'a, R, Self::RR<R>>
    where
        Self: 'a;
    fn robot_builder<'a, R: RobotFile>(&'a mut self, name: impl ToString) -> Self::RB<'a, R>;
}

pub trait AttachFrom<T> {
    fn attach_from(self, from: &mut T) -> Result<()>;
}
